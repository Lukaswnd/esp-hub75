// SPDX-License-Identifier: MIT
// @file lcd_dma.cpp
// @brief LCD RGB peripheral HUB75 for ESP32-P4 — HSYNC=LAT, Multi-Strand
//
// Zeilenstruktur (jede LCD-Zeile = ein (row, bit_plane) Paar):
//
//   ┌─────────────────┬──────────┬───────────────────────┐
//   │ Padding (BCM)   │ Fill     │ Shift (Pixeldaten)    │→ HSYNC=LAT
//   │ addr=prev_row   │ addr=cur │ addr=cur, OE_inv=0    │
//   │ OE von Bright.  │ OE=off   │ RGB = Farben          │
//   └─────────────────┴──────────┴───────────────────────┘
//   col 0                                         h_res_-1
//
//   shift_col_ = h_res_ - dma_width_  (konstant für alle Zeilen)
//
//   OE ist via GPIO-Matrix invertiert:
//     Framebuffer OE_inv=0 → GPIO HIGH → HUB75 OE=HIGH → geblankt
//     Framebuffer OE_inv=1 → GPIO LOW  → HUB75 OE=LOW  → Display an
//   Während HSYNC/VSYNC: data=0 → OE_inv=0 → geblankt ✓
//   pclk_idle_high=1 → kein Takt während Blanking → keine Spurious-Shifts ✓

#include "lcd_dma.h"
#ifdef HUB75_DMA_ENGINE_LCD

#include "../../color/color_convert.h"
#include "../../panels/scan_patterns.h"
#include "../../panels/panel_layout.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <driver/gpio.h>
#include <esp_cache.h>
#include <esp_log.h>
#include <esp_memory_utils.h>
#include <soc/gpio_struct.h>

static const char *TAG = "LcdDma";

namespace hub75 {

// ============================================================================
// Bit-Layout Konstanten
// ============================================================================
// Konsistent für 16-bit und 24-bit Modus.
// Stränge beginnen immer bei Bit 6, in 6er-Gruppen.

// Control (Byte 0, Bits 0-5)
static constexpr uint8_t ADDR_MASK  = 0x1F;
static constexpr uint8_t OE_INV_BIT = 5;
static constexpr uint8_t OE_INV_MASK = 1u << OE_INV_BIT;

// Strang-Basis-Offsets im Pixel-Wort (Bit-Position von R1)
static constexpr uint8_t STRAND_BASE[3] = {6, 12, 18};

// Innerhalb eines 6-Bit Strang-Blocks: R1=0 G1=1 B1=2 R2=3 G2=4 B2=5
static constexpr uint32_t STRAND_UPPER_BITS = 0x07;  // R1,G1,B1
static constexpr uint32_t STRAND_LOWER_BITS = 0x38;  // R2,G2,B2
static constexpr uint32_t STRAND_ALL_BITS   = 0x3F;

// Maske die NUR Control-Bits behält (für clear)
static constexpr uint32_t CTRL_KEEP_MASK = 0x3F;

// GPIO Pin-Indizes im LCD-Datenbus
static constexpr int PIN_A  = 0, PIN_B  = 1, PIN_C  = 2;
static constexpr int PIN_D  = 3, PIN_E  = 4, PIN_OE = 5;
// S1: 6-11, S2: 12-17, S3: 18-23

// Minimale Blanking-Pixel am Ende des Display-Fensters (Adress-Settling)
static constexpr size_t ADDR_SETTLE_PX = 2;


#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
// ============================================================================
// Hilfsfunktionen für virtuelle Dimensionen
// ============================================================================

static constexpr uint16_t calc_virt_w(uint8_t ns, Hub75StrandArrangement a,
                                       uint16_t sw) {
  return (ns > 1 && a == Hub75StrandArrangement::COLUMNS) ? sw * ns : sw;
}
static constexpr uint16_t calc_virt_h(uint8_t ns, Hub75StrandArrangement a,
                                       uint16_t sh) {
  return (ns > 1 && a == Hub75StrandArrangement::ROWS) ? sh * ns : sh;
}
#endif

// ============================================================================
// Constructor
// ============================================================================

LcdDma::LcdDma(const Hub75Config &config)
    : PlatformDma(config),
      panel_handle_(nullptr),
      front_idx_(0), active_idx_(0),
      is_double_buffered_(false),
      bit_depth_(HUB75_BIT_DEPTH),
      lsbMsbTransitionBit_(0),
      actual_clock_hz_(static_cast<uint32_t>(config.output_clock_speed)),
      panel_width_(config.panel_width),
      panel_height_(config.panel_height),
      layout_rows_(config.layout_rows),
      layout_cols_(config.layout_cols),
      strand_width_(config.panel_width * config.layout_cols),
      strand_height_(config.panel_height * config.layout_rows),
      dma_width_(get_effective_dma_width(config.scan_wiring, config.panel_width,
                                         config.layout_rows, config.layout_cols)),
      num_rows_(get_effective_num_rows(config.scan_wiring, config.panel_height)),
#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
      virtual_width_(calc_virt_w(std::clamp(config.num_strands, uint8_t(1), uint8_t(3)),
                                 config.strand_arrangement, config.panel_width * config.layout_cols)),
      virtual_height_(calc_virt_h(std::clamp(config.num_strands, uint8_t(1), uint8_t(3)),
                                  config.strand_arrangement, config.panel_height * config.layout_rows)),
      num_strands_(std::clamp(config.num_strands, uint8_t(1), uint8_t(3))),
      strand_arrangement_(config.strand_arrangement),
      data_width_(config.num_strands > 1 ? 24 : 16),
      bytes_per_pixel_(config.num_strands > 1 ? 3 : 2),
#else
      virtual_width_(config.panel_width * config.layout_cols),
      virtual_height_(config.panel_height * config.layout_rows),
      data_width_(16),
      bytes_per_pixel_(2),
#endif
      scan_wiring_(config.scan_wiring),
      layout_(config.layout),
      needs_scan_remap_(config.scan_wiring != Hub75ScanWiring::STANDARD_TWO_SCAN),
      needs_layout_remap_(config.layout != Hub75PanelLayout::HORIZONTAL),
      rotation_(config.rotation),
      h_res_(0), v_res_(0), line_stride_(0), fb_size_(0), shift_col_(0),
      basis_brightness_(config.brightness),
      intensity_(1.0f),
      transfer_started_(false) {
  lcd_fb_[0] = lcd_fb_[1] = nullptr;
  std::memset(bcm_padding_, 0, sizeof(bcm_padding_));
}

LcdDma::~LcdDma() { LcdDma::shutdown(); }

// ============================================================================
// Init
// ============================================================================

bool LcdDma::init() {
#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
  ESP_LOGI(TAG, "=== LCD-DMA HUB75 (HSYNC=LAT, %d-bit, %d Strang%s) ===",
           data_width_, num_strands_, num_strands_ > 1 ? "e" : "");
#else
  ESP_LOGI(TAG, "=== LCD-DMA HUB75 (HSYNC=LAT, %d-bit) ===", data_width_);
#endif
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Strang: %dx%d, Virtual: %dx%d",
           panel_width_, panel_height_, layout_cols_, layout_rows_,
           strand_width_, strand_height_, virtual_width_, virtual_height_);
  ESP_LOGI(TAG, "DMA: width=%d, rows=%d, bit_depth=%d", dma_width_, num_rows_, bit_depth_);

  // 1. BCM-Timings berechnen → h_res_, v_res_, shift_col_
  calculate_bcm_timings();

  // 2. Brightness-Koeffizienten
  init_brightness_coeffs(dma_width_, config_.latch_blanking);

  // 3. LUT für BCM-Monotonie anpassen
#if HUB75_GAMMA_MODE == 1 || HUB75_GAMMA_MODE == 2
  if (lsbMsbTransitionBit_ > 0) {
    int adj = adjust_lut_for_bcm(lut_, bit_depth_, lsbMsbTransitionBit_);
    ESP_LOGI(TAG, "LUT: %d Einträge angepasst (transition=%d)", adj, lsbMsbTransitionBit_);
  }
#endif

  // 4. GPIO Drive Strength
  configure_gpio();

  // 5. LCD-Panel erstellen (alloziert PSRAM FBs, startet DMA noch nicht)
  if (!create_lcd_panel()) return false;

  // 6. Framebuffer-Pointer holen
  if (config_.double_buffer) {
    void *fb0 = nullptr, *fb1 = nullptr;
    esp_err_t e = esp_lcd_rgb_panel_get_frame_buffer(panel_handle_, 2, &fb0, &fb1);
    if (e != ESP_OK || !fb0) {
      ESP_LOGE(TAG, "get_frame_buffer(2): %s", esp_err_to_name(e));
      shutdown(); return false;
    }
    lcd_fb_[0] = static_cast<uint8_t *>(fb0);
    lcd_fb_[1] = static_cast<uint8_t *>(fb1);
    is_double_buffered_ = (lcd_fb_[1] != nullptr);
  } else {
    void *fb0 = nullptr;
    esp_err_t e = esp_lcd_rgb_panel_get_frame_buffer(panel_handle_, 1, &fb0);
    if (e != ESP_OK || !fb0) {
      ESP_LOGE(TAG, "get_frame_buffer(1): %s", esp_err_to_name(e));
      shutdown(); return false;
    }
    lcd_fb_[0] = static_cast<uint8_t *>(fb0);
  }
  front_idx_ = 0;
  active_idx_ = is_double_buffered_ ? 1 : 0;

  // 7. Blanke Struktur aufbauen (Adressen, OE=geblankt, RGB=0)
  initialize_blank_buffer(lcd_fb_[0]);
  if (lcd_fb_[1]) initialize_blank_buffer(lcd_fb_[1]);

  // 8. Brightness-OE in Padding-Bereichen setzen
  set_brightness_oe();

  // 9. Cache → PSRAM flushen
  for (auto fb : lcd_fb_) {
#if HUB75_EXTERNAL_FRAMEBUFFERS == 1
    if (fb && esp_ptr_external_ram(fb))
      esp_cache_msync(fb, fb_size_,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif
  }

  // 10. OE-Pin vor Panel-Init auf HIGH setzen (geblankt)
  gpio_set_direction((gpio_num_t)config_.pins.oe, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)config_.pins.oe, 1);

  // 11. LCD starten
  esp_err_t e = esp_lcd_panel_reset(panel_handle_);
  if (e != ESP_OK) ESP_LOGW(TAG, "panel_reset: %s", esp_err_to_name(e));

  e = esp_lcd_panel_init(panel_handle_);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "panel_init: %s", esp_err_to_name(e));
    shutdown(); return false;
  }

  // 12. KRITISCH: OE-Pin via GPIO-Matrix invertieren
  //     esp_lcd_panel_init() hat PIN_OE als LCD-Datenausgang konfiguriert.
  //     Ohne Invertierung: OE_inv=0 im FB → GPIO LOW → HUB75 OE=LOW → Display AN → Ghosting!
  //     Mit Invertierung:  OE_inv=0 → GPIO HIGH → HUB75 OE=HIGH → geblankt ✓
  {
    GPIO.func_out_sel_cfg[config_.pins.oe].out_inv_sel = 1;
    ESP_LOGI(TAG, "OE GPIO %d invertiert via GPIO-Matrix", config_.pins.oe);;
  }

  transfer_started_ = true;

  const float hz_est = static_cast<float>(actual_clock_hz_)
      / ((h_res_ + 3.0f) * (v_res_ + 1.0f));
  ESP_LOGI(TAG, "Läuft: H=%d V=%d shift_col=%zu %.1fKB/fb ~%.0fHz %s",
           h_res_, v_res_, shift_col_, fb_size_ / 1024.0f, hz_est,
           is_double_buffered_ ? "(double)" : "(single)");
  return true;
}

void LcdDma::shutdown() {
  if (panel_handle_) {
    esp_lcd_panel_del(panel_handle_);
    panel_handle_ = nullptr;
  }
  lcd_fb_[0] = lcd_fb_[1] = nullptr;
  transfer_started_ = false;
}

// ============================================================================
// GPIO
// ============================================================================

void LcdDma::configure_gpio() {
  auto set_drive = [](int pin) {
    if (pin >= 0) gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
  };
  set_drive(config_.pins.r1);  set_drive(config_.pins.g1);  set_drive(config_.pins.b1);
  set_drive(config_.pins.r2);  set_drive(config_.pins.g2);  set_drive(config_.pins.b2);
  set_drive(config_.pins.a);   set_drive(config_.pins.b);   set_drive(config_.pins.c);
  set_drive(config_.pins.d);   set_drive(config_.pins.e);
  set_drive(config_.pins.lat); set_drive(config_.pins.oe);  set_drive(config_.pins.clk);
#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
  if (num_strands_ >= 2) {
    set_drive(config_.strand2_pins.r1); set_drive(config_.strand2_pins.g1);
    set_drive(config_.strand2_pins.b1); set_drive(config_.strand2_pins.r2);
    set_drive(config_.strand2_pins.g2); set_drive(config_.strand2_pins.b2);
  }
  if (num_strands_ >= 3) {
    set_drive(config_.strand3_pins.r1); set_drive(config_.strand3_pins.g1);
    set_drive(config_.strand3_pins.b1); set_drive(config_.strand3_pins.r2);
    set_drive(config_.strand3_pins.g2); set_drive(config_.strand3_pins.b2);
  }
#endif
}

// ============================================================================
// LCD-Panel
// ============================================================================

bool LcdDma::create_lcd_panel() {
  esp_lcd_rgb_panel_config_t cfg{};
  cfg.clk_src         = LCD_CLK_SRC_DEFAULT;
  cfg.timings.pclk_hz = actual_clock_hz_;
  cfg.timings.h_res   = h_res_;
  cfg.timings.v_res   = v_res_;

  // HSYNC = LAT: 1-PCLK Puls, minimale Porches
  cfg.timings.hsync_pulse_width  = 1;
  cfg.timings.hsync_back_porch   = 1;
  cfg.timings.hsync_front_porch  = 1;
  cfg.timings.vsync_pulse_width  = 1;
  cfg.timings.vsync_back_porch   = 0;
  cfg.timings.vsync_front_porch  = 0;

  // Clock: idle HIGH → keine Spurious-Shifts während Blanking
  // HSYNC idle LOW, puls HIGH → HUB75 LAT active-HIGH ✓
  cfg.timings.flags.pclk_idle_high  = 1;
  cfg.timings.flags.pclk_active_neg = config_.clk_phase_inverted ? 0 : 1;
  cfg.timings.flags.hsync_idle_low  = 1;
  cfg.timings.flags.vsync_idle_low  = 1;
  cfg.timings.flags.de_idle_high    = 1;

  cfg.data_width     = data_width_;
  cfg.bits_per_pixel = data_width_;  // 1:1 Mapping, kein Padding
  cfg.num_fbs        = config_.double_buffer ? 2 : 1;
  cfg.bounce_buffer_size_px = 0;

  // HSYNC → LAT, PCLK → CLK, Rest unbenutzt
  cfg.hsync_gpio_num = (gpio_num_t)config_.pins.lat;
  cfg.vsync_gpio_num = GPIO_NUM_NC;
  cfg.de_gpio_num    = GPIO_NUM_NC;
  cfg.pclk_gpio_num  = (gpio_num_t)config_.pins.clk;
  cfg.disp_gpio_num  = GPIO_NUM_NC;

  // Daten-Pins initialisieren
  std::memset(cfg.data_gpio_nums, -1, sizeof(cfg.data_gpio_nums));

  // Control (Bits 0-5)
  cfg.data_gpio_nums[PIN_A]  = (gpio_num_t)config_.pins.a;
  cfg.data_gpio_nums[PIN_B]  = (gpio_num_t)config_.pins.b;
  cfg.data_gpio_nums[PIN_C]  = (gpio_num_t)config_.pins.c;
  cfg.data_gpio_nums[PIN_D]  = (gpio_num_t)config_.pins.d;
  cfg.data_gpio_nums[PIN_E]  = (gpio_num_t)config_.pins.e;
  cfg.data_gpio_nums[PIN_OE] = (gpio_num_t)config_.pins.oe;

  // Strang 1 (Bits 6-11, immer)
  cfg.data_gpio_nums[6]  = (gpio_num_t)config_.pins.r1;
  cfg.data_gpio_nums[7]  = (gpio_num_t)config_.pins.g1;
  cfg.data_gpio_nums[8]  = (gpio_num_t)config_.pins.b1;
  cfg.data_gpio_nums[9]  = (gpio_num_t)config_.pins.r2;
  cfg.data_gpio_nums[10] = (gpio_num_t)config_.pins.g2;
  cfg.data_gpio_nums[11] = (gpio_num_t)config_.pins.b2;

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
  // Strang 2 (Bits 12-17, nur 24-bit)
  if (num_strands_ >= 2) {
    cfg.data_gpio_nums[12] = (gpio_num_t)config_.strand2_pins.r1;
    cfg.data_gpio_nums[13] = (gpio_num_t)config_.strand2_pins.g1;
    cfg.data_gpio_nums[14] = (gpio_num_t)config_.strand2_pins.b1;
    cfg.data_gpio_nums[15] = (gpio_num_t)config_.strand2_pins.r2;
    cfg.data_gpio_nums[16] = (gpio_num_t)config_.strand2_pins.g2;
    cfg.data_gpio_nums[17] = (gpio_num_t)config_.strand2_pins.b2;
  }

  // Strang 3 (Bits 18-23, nur 24-bit)
  if (num_strands_ >= 3) {
    cfg.data_gpio_nums[18] = (gpio_num_t)config_.strand3_pins.r1;
    cfg.data_gpio_nums[19] = (gpio_num_t)config_.strand3_pins.g1;
    cfg.data_gpio_nums[20] = (gpio_num_t)config_.strand3_pins.b1;
    cfg.data_gpio_nums[21] = (gpio_num_t)config_.strand3_pins.r2;
    cfg.data_gpio_nums[22] = (gpio_num_t)config_.strand3_pins.g2;
    cfg.data_gpio_nums[23] = (gpio_num_t)config_.strand3_pins.b2;
  }
#endif

  // Speicher
#if HUB75_EXTERNAL_FRAMEBUFFERS == 1
  cfg.flags.fb_in_psram       = 1;
#else
  cfg.flags.fb_in_psram       = 0;
#endif
  cfg.flags.double_fb         = config_.double_buffer ? 1u : 0u;
  cfg.flags.no_fb             = 0;
  cfg.flags.refresh_on_demand = 0;
  cfg.flags.bb_invalidate_cache = 0;

  esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_lcd_new_rgb_panel: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "LCD Panel erstellt: %d-bit, HSYNC=GPIO%d→LAT", data_width_, config_.pins.lat);
  return true;
}

// ============================================================================
// BCM Timing
// ============================================================================

size_t LcdDma::calculate_bcm_padding(uint8_t bit_plane) const {
  const size_t base_pad = config_.latch_blanking;
  const size_t base_disp = dma_width_ - base_pad;
  if (bit_plane <= lsbMsbTransitionBit_)
    return base_pad + base_disp;
  const size_t reps = 1u << (bit_plane - lsbMsbTransitionBit_ - 1);
  return base_pad + reps * base_disp;
}

void LcdDma::calculate_bcm_timings() {
  const uint32_t target_hz = config_.min_refresh_rate;
  static constexpr uint16_t H_OVERHEAD = 3;  // front+hsync+back porch
  static constexpr uint16_t V_OVERHEAD = 1;

  lsbMsbTransitionBit_ = 0;
  int best_hz = 0;

  while (true) {
    size_t max_line = 0;
    for (int b = 0; b < bit_depth_; b++) {
      max_line = std::max(max_line, dma_width_ + calculate_bcm_padding(b));
    }
    const float frame_s = float((max_line + H_OVERHEAD)
        * (uint32_t(num_rows_) * bit_depth_ + V_OVERHEAD))

        / float(actual_clock_hz_);
    best_hz = int(1.0f / frame_s);

    if (best_hz >= int(target_hz)) break;
    if (lsbMsbTransitionBit_ < bit_depth_ - 1) {
      lsbMsbTransitionBit_++;
    } else {
      ESP_LOGW(TAG, "Max %d Hz (Ziel %lu)", best_hz, (unsigned long)target_hz);
      break;
    }
  }

  // Finale Werte committen
  size_t max_line = 0;
  for (int b = 0; b < bit_depth_; b++) {
    bcm_padding_[b] = calculate_bcm_padding(b);
    max_line = std::max(max_line, dma_width_ + bcm_padding_[b]);
  }

  h_res_       = static_cast<uint16_t>(max_line);
  v_res_       = num_rows_ * bit_depth_;
  line_stride_ = size_t(h_res_) * bytes_per_pixel_;
  fb_size_     = line_stride_ * v_res_;
  shift_col_   = h_res_ - dma_width_;

  // Waste berechnen
  size_t useful = 0;
  for (int b = 0; b < bit_depth_; b++)
    useful += (dma_width_ + bcm_padding_[b]) * num_rows_;
  const float waste = (1.0f - float(useful) / (h_res_ * v_res_)) * 100.0f;

  ESP_LOGI(TAG, "BCM: transition=%d h=%d v=%d shift=%zu fb=%.1fKB waste=%.0f%% ~%dHz",
           lsbMsbTransitionBit_, h_res_, v_res_, shift_col_,
           fb_size_ / 1024.0f, waste, best_hz);
}

// ============================================================================
// Buffer-Initialisierung
// ============================================================================
//
// Jede LCD-Zeile (row, bit) hat folgende Struktur:
//
//   [Padding: bcm_padding_[prev_bit] Pixel]  ← Display vorheriger Daten
//     Hauptteil: addr=prev_row, OE_inv=0 (Brightness setzt OE_inv=1)
//     Guard:     addr=curr_row, OE_inv=0 (immer geblankt)
//   [Fill: bis shift_col_]                   ← addr=curr_row, geblankt
//   [Shift: dma_width_ Pixel]                ← addr=curr_row, geblankt, RGB=Daten
//   → HSYNC=LAT (Hardware-Latch)

void LcdDma::initialize_blank_buffer(uint8_t *fb) {
  if (!fb) return;
  std::memset(fb, 0, fb_size_);

  for (int row = 0; row < num_rows_; row++) {
    const uint8_t curr_addr = row & ADDR_MASK;

    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);

      // Vorherige Zeile bestimmen (deren Daten wir im Padding anzeigen)
      const int prev_bit = (bit == 0) ? (bit_depth_ - 1) : (bit - 1);
      const int prev_row = (bit == 0)
          ? ((row == 0) ? (num_rows_ - 1) : (row - 1))
          : row;
      const uint8_t prev_addr = prev_row & ADDR_MASK;

      const size_t padding = bcm_padding_[prev_bit];
      const size_t guard = std::min(size_t(config_.latch_blanking), padding);
      const size_t main_pad = padding - guard;

      size_t col = 0;

      // Padding Hauptteil: addr=prev_row, OE_inv=0 (geblankt)
      for (size_t i = 0; i < main_pad; i++, col++)
        pixel_at(fb, line, col)[0] = prev_addr;  // OE_inv=0

      // Padding Guard: addr→curr_row, OE_inv=0 (immer geblankt)
      for (size_t i = 0; i < guard; i++, col++)
        pixel_at(fb, line, col)[0] = curr_addr;

      // Fill: geblankt, curr_addr
      for (; col < shift_col_; col++)
        pixel_at(fb, line, col)[0] = curr_addr;

      // Shift: curr_addr, OE_inv=0, RGB=0 (wird von draw/fill gesetzt)
      for (size_t i = 0; i < dma_width_; i++, col++)
        pixel_at(fb, line, col)[0] = curr_addr;

      assert(col == h_res_);
    }
  }
}

// ============================================================================
// Brightness / OE
// ============================================================================
// Nur der Padding-Hauptteil (addr=prev_row) wird modifiziert.
// Guard, Fill, Shift bleiben immer OE_inv=0 (geblankt).

void LcdDma::set_brightness_oe_buffer(uint8_t *fb, uint8_t brightness) {
  if (!fb) return;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);
      const int prev_bit = (bit == 0) ? (bit_depth_ - 1) : (bit - 1);
      const size_t padding = bcm_padding_[prev_bit];

      // 1. Alle Padding-OE zurücksetzen (geblankt)
      for (size_t i = 0; i < padding; i++)
        pixel_at(fb, line, i)[0] &= ~OE_INV_MASK;

      if (brightness == 0 || padding == 0) continue;

      // 2. Verfügbare Display-Region (ohne Guard)
      const size_t guard = std::min(size_t(config_.latch_blanking), padding);
      const int avail = int(padding - guard);
      if (avail < 2) continue;

      // 3. BCM-Differenzierung (identisch zu parlio_dma.cpp)
      int max_display;
      if (prev_bit <= lsbMsbTransitionBit_) {
        const int bitplane = bit_depth_ - 1 - prev_bit;
        const int bitshift = (bit_depth_ - lsbMsbTransitionBit_ - 1) >> 1;
        const int rightshift = std::max(bitplane - bitshift - 2, 0);
        max_display = avail >> rightshift;
      } else {
        max_display = avail;
      }
      if (max_display < 2) continue;

      // 4. Quadratisches Brightness-Remapping
      const int eff = remap_brightness(brightness);
      int display_count = (max_display * eff) >> 8;

      // Graduelles Minimum für Randwerte
      const int min_bit = std::max(0, bit_depth_ - 1 - (eff >> 4));
      if (eff > 0 && display_count == 0 && prev_bit >= min_bit)
        display_count = 1;

      // Sicherheit: mindestens 1 Pixel geblankt
      display_count = std::min(display_count, max_display - 1);

      // 5. Display-Fenster zentrieren, Adress-Settling beachten
      size_t start = (avail - display_count) / 2;
      start = std::max(start, ADDR_SETTLE_PX);
      if (start + display_count > size_t(avail))
        display_count = avail - int(start);
      if (display_count <= 0) continue;

      // 6. OE_inv=1 setzen (Display an)
      for (int i = 0; i < display_count; i++)
        pixel_at(fb, line, start + i)[0] |= OE_INV_MASK;
    }
  }
}

void LcdDma::set_brightness_oe() {
  const uint8_t bri = uint8_t(float(basis_brightness_) * intensity_);
  for (auto fb : lcd_fb_)
    if (fb) set_brightness_oe_buffer(fb, bri);
  flush_cache();
}

void LcdDma::flush_cache() {
#if HUB75_EXTERNAL_FRAMEBUFFERS == 1
  uint8_t *fb = lcd_fb_[active_idx_];
  if (fb && esp_ptr_external_ram(fb))
    esp_cache_msync(fb, fb_size_,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif
}

// ============================================================================
// Pixel-Zeichnung
// ============================================================================
// Zwei Pfade:
//   Fast-Path: ROTATE_0, kein Layout/Scan-Remap → Pro-Zeile Vorberechnung,
//              innere Schleife ohne Koordinaten-Transform
//   General-Path: Voller Transform pro Pixel

HUB75_IRAM void LcdDma::draw_pixels(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h,
    const uint8_t *buffer, Hub75PixelFormat format,
    Hub75ColorOrder color_order, bool big_endian) {

  uint8_t *fb = lcd_fb_[active_idx_];
  if (!fb || !buffer) [[unlikely]] return;

  const uint16_t rw = RotationTransform::get_rotated_width(virtual_width_, virtual_height_, rotation_);
  const uint16_t rh = RotationTransform::get_rotated_height(virtual_width_, virtual_height_, rotation_);
  if (x >= rw || y >= rh) [[unlikely]] return;
  if (x + w > rw) w = rw - x;
  if (y + h > rh) h = rh - y;

  const size_t stride = (format == Hub75PixelFormat::RGB888) ? 3
                      : (format == Hub75PixelFormat::RGB565) ? 2 : 4;

  const bool fast = (rotation_ == Hub75Rotation::ROTATE_0)
                    && !needs_layout_remap_ && !needs_scan_remap_;

  const uint8_t *ptr = buffer;

  for (uint16_t dy = 0; dy < h; dy++) {
    const uint16_t py = y + dy;

    if (fast) {
      // --- Pro-Zeile Vorberechnung ---
      uint8_t sid = 0;
      uint16_t local_y = py;

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
      if (num_strands_ > 1 && strand_arrangement_ == Hub75StrandArrangement::ROWS) {
        sid = py / strand_height_;
        local_y = py % strand_height_;
      }
#endif
      const bool is_lower = local_y >= num_rows_;
      const uint16_t row = is_lower ? (local_y - num_rows_) : local_y;
      const int base_line = row * bit_depth_;

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
      // Strang-Bit-Offset für diese Zeile (ROWS: sid fest, COLUMNS: pro Pixel)
      const bool rows_mode = (num_strands_ <= 1
                              || strand_arrangement_ == Hub75StrandArrangement::ROWS);
#else
      const bool rows_mode = true;
#endif

      uint8_t row_offset, row_base;
      uint32_t row_clr;
      if (rows_mode) {
        row_base = STRAND_BASE[sid];
        row_offset = row_base + (is_lower ? 3 : 0);
        row_clr = 0x7u << row_offset;
      }

      // --- Innere Pixel-Schleife ---
      for (uint16_t dx = 0; dx < w; dx++) {
        uint16_t px = x + dx;
        uint16_t dma_x = px;
        uint8_t offset, base;
        uint32_t clr;

        if (rows_mode) {
          offset = row_offset;
          base = row_base;
          clr = row_clr;
        } else {
          // COLUMNS: strand_id hängt von x ab
          uint8_t s = px / strand_width_;
          dma_x = px - s * strand_width_;
          base = STRAND_BASE[s];
          offset = base + (is_lower ? 3 : 0);
          clr = 0x7u << offset;
        }

        uint8_t r8, g8, b8;
        extract_rgb888_from_format(ptr, 0, format, color_order, big_endian, r8, g8, b8);
        ptr += stride;

        const uint16_t rc = lut_[r8], gc = lut_[g8], bc = lut_[b8];

        for (int bit = 0; bit < bit_depth_; bit++) {
          uint32_t rgb = uint32_t((rc >> bit) & 1)
                       | (uint32_t((gc >> bit) & 1) << 1)
                       | (uint32_t((bc >> bit) & 1) << 2);

          uint8_t *p = pixel_at(fb, base_line + bit, shift_col_ + dma_x);
          write_px(p, (read_px(p) & ~clr) | (rgb << offset));
        }
      }

    } else {
      // --- General-Path: voller Transform pro Pixel ---
      for (uint16_t dx = 0; dx < w; dx++) {
        uint16_t px = x + dx;
        Coords c = {px, py};

        // Rotation
        if (rotation_ != Hub75Rotation::ROTATE_0)
          c = RotationTransform::apply(c, rotation_, virtual_width_, virtual_height_);

        // Strang-Mapping
        uint8_t sid = 0;
#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
        if (num_strands_ > 1) {
          if (strand_arrangement_ == Hub75StrandArrangement::ROWS) {
            sid = c.y / strand_height_;
            c.y = c.y % strand_height_;
          } else {
            sid = c.x / strand_width_;
            c.x = c.x % strand_width_;
          }
        }
#endif

        // Panel-Layout
        if (needs_layout_remap_)
          c = PanelLayoutRemap::remap(c, layout_, panel_width_, panel_height_,
                                      layout_rows_, layout_cols_);
        // Scan-Pattern
        if (needs_scan_remap_)
          c = ScanPatternRemap::remap(c, scan_wiring_, panel_width_, panel_height_);

        const bool is_lower = c.y >= num_rows_;
        const uint16_t row = is_lower ? (c.y - num_rows_) : c.y;
        const uint8_t base = STRAND_BASE[sid];
        const uint8_t offset = base + (is_lower ? 3 : 0);
        const uint32_t clr = 0x7u << offset;
        const int base_line = row * bit_depth_;

        uint8_t r8, g8, b8;
        extract_rgb888_from_format(ptr, 0, format, color_order, big_endian, r8, g8, b8);
        ptr += stride;

        const uint16_t rc = lut_[r8], gc = lut_[g8], bc = lut_[b8];

        for (int bit = 0; bit < bit_depth_; bit++) {
          uint32_t rgb = uint32_t((rc >> bit) & 1)
                       | (uint32_t((gc >> bit) & 1) << 1)
                       | (uint32_t((bc >> bit) & 1) << 2);
          uint8_t *p = pixel_at(fb, base_line + bit, shift_col_ + c.x);
          write_px(p, (read_px(p) & ~clr) | (rgb << offset));
        }
      }
    }
  }

  if (!is_double_buffered_) flush_cache();
}

// ============================================================================
// Clear
// ============================================================================

void LcdDma::clear() {
  uint8_t *fb = lcd_fb_[active_idx_];
  if (!fb) return;

  // Alle RGB-Bits (6+) löschen, Control (0-5) behalten
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);
      for (int col = 0; col < dma_width_; col++) {
        uint8_t *p = pixel_at(fb, line, shift_col_ + col);
        // Für 16-bit: Bits 6-15 löschen. Für 24-bit: Bits 6-23 löschen.
        // In beiden Fällen: byte[0] &= 0x3F, rest = 0.
        p[0] &= 0x3F;
        p[1] = 0;
        if (bytes_per_pixel_ >= 3) p[2] = 0;
      }
    }
  }

  if (!is_double_buffered_) flush_cache();
}

// ============================================================================
// Fill
// ============================================================================

HUB75_IRAM void LcdDma::fill(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h,
    uint8_t r, uint8_t g, uint8_t b) {

  uint8_t *fb = lcd_fb_[active_idx_];
  if (!fb) [[unlikely]] return;

  const uint16_t rw = RotationTransform::get_rotated_width(virtual_width_, virtual_height_, rotation_);
  const uint16_t rh = RotationTransform::get_rotated_height(virtual_width_, virtual_height_, rotation_);
  if (x >= rw || y >= rh) [[unlikely]] return;
  if (x + w > rw) w = rw - x;
  if (y + h > rh) h = rh - y;

  // Farb-LUT einmal anwenden
  const uint16_t rc = lut_[r], gc = lut_[g], bc = lut_[b];

  // Bit-Patterns pro Strang × Bitebene × Hälfte vorberechnen
  // patterns[strand][bit] = 3-Bit RGB Wert (unverschoben)
  uint32_t raw_pat[HUB75_MAX_BIT_DEPTH];
  for (int bit = 0; bit < bit_depth_; bit++) {
    raw_pat[bit] = uint32_t((rc >> bit) & 1)
                 | (uint32_t((gc >> bit) & 1) << 1)
                 | (uint32_t((bc >> bit) & 1) << 2);
  }

  const bool fast = (rotation_ == Hub75Rotation::ROTATE_0)
                    && !needs_layout_remap_ && !needs_scan_remap_;

  for (uint16_t dy = 0; dy < h; dy++) {
    const uint16_t py = y + dy;

    if (fast) {
      uint8_t sid = 0;
      uint16_t local_y = py;

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
      if (num_strands_ > 1 && strand_arrangement_ == Hub75StrandArrangement::ROWS) {
        sid = py / strand_height_;
        local_y = py % strand_height_;
      }
#endif

      const bool is_lower = local_y >= num_rows_;
      const uint16_t row = is_lower ? (local_y - num_rows_) : local_y;
      const int base_line = row * bit_depth_;

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
      const bool rows_mode = (num_strands_ <= 1
                              || strand_arrangement_ == Hub75StrandArrangement::ROWS);
#else
      const bool rows_mode = true;
#endif

      // Für ROWS: Offset+Patterns einmal pro Zeile
      uint32_t shifted_pat[HUB75_MAX_BIT_DEPTH];
      uint32_t clr = 0;
      if (rows_mode) {
        const uint8_t off = STRAND_BASE[sid] + (is_lower ? 3 : 0);
        clr = 0x7u << off;
        for (int bit = 0; bit < bit_depth_; bit++)
          shifted_pat[bit] = raw_pat[bit] << off;
      }

      for (uint16_t dx = 0; dx < w; dx++) {
        uint16_t px = x + dx;
        uint16_t dma_x = px;

        const uint32_t *pat = shifted_pat;
        uint32_t c_clr = clr;

        if (!rows_mode) {
          // COLUMNS: Strang-Offset hängt von x ab
          uint8_t s = px / strand_width_;
          dma_x = px - s * strand_width_;
          uint8_t off = STRAND_BASE[s] + (is_lower ? 3 : 0);
          c_clr = 0x7u << off;
          // Patterns on-the-fly shiften (kein Array nötig da gleiche raw_pat)
          for (int bit = 0; bit < bit_depth_; bit++) {
            uint8_t *p = pixel_at(fb, base_line + bit, shift_col_ + dma_x);
            write_px(p, (read_px(p) & ~c_clr) | (raw_pat[bit] << off));
          }
          continue;
        }

        for (int bit = 0; bit < bit_depth_; bit++) {
          uint8_t *p = pixel_at(fb, base_line + bit, shift_col_ + dma_x);
          write_px(p, (read_px(p) & ~c_clr) | pat[bit]);
        }
      }

    } else {
      // General-Path (identisch zu draw_pixels, aber mit konstantem RGB)
      for (uint16_t dx = 0; dx < w; dx++) {
        uint16_t px = x + dx;
        Coords c = {px, py};

        if (rotation_ != Hub75Rotation::ROTATE_0)
          c = RotationTransform::apply(c, rotation_, virtual_width_, virtual_height_);

        uint8_t sid = 0;
#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
        if (num_strands_ > 1) {
          if (strand_arrangement_ == Hub75StrandArrangement::ROWS) {
            sid = c.y / strand_height_;
            c.y = c.y % strand_height_;
          } else {
            sid = c.x / strand_width_;
            c.x = c.x % strand_width_;
          }
        }
#endif

        if (needs_layout_remap_)
          c = PanelLayoutRemap::remap(c, layout_, panel_width_, panel_height_,
                                      layout_rows_, layout_cols_);
        if (needs_scan_remap_)
          c = ScanPatternRemap::remap(c, scan_wiring_, panel_width_, panel_height_);

        const bool is_lower = c.y >= num_rows_;
        const uint16_t row = is_lower ? (c.y - num_rows_) : c.y;
        const uint8_t off = STRAND_BASE[sid] + (is_lower ? 3 : 0);
        const uint32_t c_clr = 0x7u << off;
        const int base_line = row * bit_depth_;

        for (int bit = 0; bit < bit_depth_; bit++) {
          uint8_t *p = pixel_at(fb, base_line + bit, shift_col_ + c.x);
          write_px(p, (read_px(p) & ~c_clr) | (raw_pat[bit] << off));
        }
      }
    }
  }

  if (!is_double_buffered_) flush_cache();
}

// ============================================================================
// Double Buffering
// ============================================================================

void LcdDma::flip_buffer() {
  if (!is_double_buffered_) return;
  flush_cache();
  esp_lcd_panel_draw_bitmap(panel_handle_, 0, 0, h_res_, v_res_, lcd_fb_[active_idx_]);
  std::swap(front_idx_, active_idx_);
}

// ============================================================================
// Transfer Control
// ============================================================================

void LcdDma::start_transfer()  { transfer_started_ = true; }
void LcdDma::stop_transfer()   { transfer_started_ = false; }

// ============================================================================
// Brightness / Intensity / Rotation
// ============================================================================

void LcdDma::set_basis_brightness(uint8_t brightness) {
  if (brightness != basis_brightness_) {
    basis_brightness_ = brightness;
    set_brightness_oe();
  }
}

void LcdDma::set_intensity(float intensity) {
  intensity = std::clamp(intensity, 0.0f, 1.0f);
  if (intensity != intensity_) {
    intensity_ = intensity;
    set_brightness_oe();
  }
}

void LcdDma::set_rotation(Hub75Rotation rotation) { rotation_ = rotation; }

}  // namespace hub75
#endif  // CONFIG_IDF_TARGET_ESP32P4