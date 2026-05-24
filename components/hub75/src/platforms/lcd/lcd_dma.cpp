// SPDX-License-Identifier: MIT
//
// @file lcd_dma.cpp
// @brief LCD RGB peripheral HUB75 implementation for ESP32-P4 (fixed)
//
// ┌─────────────────────────────────────────────────────────────────┐
// │  HSYNC = LAT approach — with shift-register compensation       │
// │                                                                │
// │  Problem: The LCD peripheral blanks all data pins during       │
// │  HSYNC. With pclk_idle_high=1, no PCLK edges occur during     │
// │  HSYNC, so panels needing CLK+LAT won't latch.                │
// │                                                                │
// │  Solution: Use pclk_idle_high=0 so PCLK transitions from      │
// │  active toggling to idle-LOW at the end of the active line.    │
// │  Configure hsync_front_porch=0 (ESP-IDF ≥5.3 allows this on   │
// │  P4) so HSYNC fires immediately. The HSYNC pulse width of 1   │
// │  PCLK produces exactly 1 CLK edge while HSYNC(=LAT) is HIGH.  │
// │                                                                │
// │  That single extra clock shifts one zero into the shift        │
// │  register. We compensate by extending the shift section by 1   │
// │  and placing pixel data offset by 1 column. The "sacrificial"  │
// │  column at position 0 (first clocked in, ends up at the far   │
// │  end of the SR) is the one pushed out by the extra clock.      │
// │                                                                │
// │  Back-porch clocks happen AFTER latch and don't matter.        │
// │                                                                │
// │  OE is inverted via GPIO matrix so data=0 during blanking      │
// │  means OE_INV=0 → GPIO HIGH → HUB75 blanked.                  │
// │                                                                │
// │  Line structure (per row, per bit-plane):                      │
// │                                                                │
// │  ┌────────────────┬────────┬─────────────────────────┐         │
// │  │ Padding (BCM   │ Fill   │ Shift (dma_width+1 px)  │→HSYNC  │
// │  │ display time)  │(blanked│ (blanked, pixel data)   │ =LAT   │
// │  │ addr=prev_row  │ filler)│ addr=curr_row           │         │
// │  └────────────────┴────────┴─────────────────────────┘         │
// │  OE_INV controlled         OE_INV=0 (off)                     │
// │  by brightness                                                 │
// └─────────────────────────────────────────────────────────────────┘

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
// 16-bit word layout constants
// ============================================================================
static constexpr uint8_t ADDR_MASK   = 0x1F;
static constexpr uint8_t OE_INV_BIT  = 5;
static constexpr uint8_t OE_INV_MASK = 1u << OE_INV_BIT;

// Byte 1: Panel 1 RGB bit positions (relative to byte 1)
static constexpr uint8_t P1_R1 = 0, P1_G1 = 1, P1_B1 = 2;
static constexpr uint8_t P1_R2 = 3, P1_G2 = 4, P1_B2 = 5;
static constexpr uint8_t P1_UPPER_MASK = (1u<<P1_R1)|(1u<<P1_G1)|(1u<<P1_B1);
static constexpr uint8_t P1_LOWER_MASK = (1u<<P1_R2)|(1u<<P1_G2)|(1u<<P1_B2);
static constexpr uint8_t P1_RGB_MASK   = P1_UPPER_MASK | P1_LOWER_MASK;

// Data bus pin indices
static constexpr int PIN_A     =  0;
static constexpr int PIN_B     =  1;
static constexpr int PIN_C     =  2;
static constexpr int PIN_D     =  3;
static constexpr int PIN_E     =  4;
static constexpr int PIN_OE    =  5;
static constexpr int PIN_P1_R1 =  8;
static constexpr int PIN_P1_G1 =  9;
static constexpr int PIN_P1_B1 = 10;
static constexpr int PIN_P1_R2 = 11;
static constexpr int PIN_P1_G2 = 12;
static constexpr int PIN_P1_B2 = 13;

static constexpr size_t ADDR_SETTLE_PIXELS = 2;

/// Maximum h_res supported by ESP32-P4 LCD_CAM (12-bit register)
static constexpr uint16_t LCD_MAX_H_RES = 4095;

// ============================================================================
// Constructor / Destructor
// ============================================================================

LcdDma::LcdDma(const Hub75Config &config)
    : PlatformDma(config),
      panel_handle_(nullptr),
      front_idx_(0),
      active_idx_(0),
      is_double_buffered_(false),
      bit_depth_(HUB75_BIT_DEPTH),
      lsbMsbTransitionBit_(0),
      actual_clock_hz_(static_cast<uint32_t>(config.output_clock_speed)),
      panel_width_(config.panel_width),
      panel_height_(config.panel_height),
      layout_rows_(config.layout_rows),
      layout_cols_(config.layout_cols),
      virtual_width_(config.panel_width * config.layout_cols),
      virtual_height_(config.panel_height * config.layout_rows),
      dma_width_(get_effective_dma_width(config.scan_wiring, config.panel_width,
                                         config.layout_rows, config.layout_cols)),
      scan_wiring_(config.scan_wiring),
      layout_(config.layout),
      needs_scan_remap_(config.scan_wiring != Hub75ScanWiring::STANDARD_TWO_SCAN),
      needs_layout_remap_(config.layout != Hub75PanelLayout::HORIZONTAL),
      rotation_(config.rotation),
      num_rows_(get_effective_num_rows(config.scan_wiring, config.panel_height)),
      h_res_(0), v_res_(0), line_stride_(0), fb_size_(0),
      shift_col_(0), blanking_clocks_before_latch_(0),
      basis_brightness_(config.brightness),
      intensity_(1.0f),
      transfer_started_(false) {
  lcd_fb_[0] = nullptr;
  lcd_fb_[1] = nullptr;
  std::memset(bcm_padding_, 0, sizeof(bcm_padding_));
}

LcdDma::~LcdDma() { LcdDma::shutdown(); }

// ============================================================================
// Initialization
// ============================================================================

bool LcdDma::init() {
  ESP_LOGI(TAG, "=== LCD RGB DMA for HUB75 (HSYNC=LAT, 16-bit) ===");
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d",
           panel_width_, panel_height_, layout_cols_, layout_rows_,
           virtual_width_, virtual_height_);
  ESP_LOGI(TAG, "DMA: width=%d, rows=%d, bit_depth=%d", dma_width_, num_rows_, bit_depth_);

  calculate_bcm_timings();
  init_brightness_coeffs(dma_width_, config_.latch_blanking);

#if HUB75_GAMMA_MODE == 1 || HUB75_GAMMA_MODE == 2
  if (lsbMsbTransitionBit_ > 0) {
    int adj = adjust_lut_for_bcm(lut_, bit_depth_, lsbMsbTransitionBit_);
    ESP_LOGI(TAG, "Adjusted %d LUT entries (transition=%d)", adj, lsbMsbTransitionBit_);
  }
#endif

  configure_gpio();

  if (!create_lcd_panel()) {
    ESP_LOGE(TAG, "Failed to create LCD panel");
    return false;
  }

  // Obtain framebuffer pointers
  if (config_.double_buffer) {
    void *fb0 = nullptr, *fb1 = nullptr;
    esp_err_t err = esp_lcd_rgb_panel_get_frame_buffer(panel_handle_, 2, &fb0, &fb1);
    if (err != ESP_OK || !fb0) {
      ESP_LOGE(TAG, "get_frame_buffer(2) failed: %s", esp_err_to_name(err));
      shutdown(); return false;
    }
    lcd_fb_[0] = static_cast<uint8_t*>(fb0);
    lcd_fb_[1] = static_cast<uint8_t*>(fb1);
    is_double_buffered_ = (lcd_fb_[1] != nullptr);
  } else {
    void *fb0 = nullptr;
    esp_err_t err = esp_lcd_rgb_panel_get_frame_buffer(panel_handle_, 1, &fb0);
    if (err != ESP_OK || !fb0) {
      ESP_LOGE(TAG, "get_frame_buffer(1) failed: %s", esp_err_to_name(err));
      shutdown(); return false;
    }
    lcd_fb_[0] = static_cast<uint8_t*>(fb0);
  }

  front_idx_  = 0;
  active_idx_ = is_double_buffered_ ? 1 : 0;

  ESP_LOGI(TAG, "Framebuffers: fb[0]=%p fb[1]=%p (%s)",
           lcd_fb_[0], lcd_fb_[1],
           is_double_buffered_ ? "double" : "single");

  // Initialize blank buffers and brightness
  initialize_blank_buffer(lcd_fb_[0]);
  if (lcd_fb_[1]) initialize_blank_buffer(lcd_fb_[1]);
  set_brightness_oe();

  // Flush cache before DMA starts
  for (auto fb : lcd_fb_) {
    if (fb && esp_ptr_external_ram(fb)) {
      esp_cache_msync(fb, fb_size_,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
  }

  // Pre-blank OE before LCD takes over GPIO
  gpio_set_direction((gpio_num_t)config_.pins.oe, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)config_.pins.oe, 1);

  // Start LCD (resets peripheral, configures timing, starts DMA)
  esp_err_t err = esp_lcd_panel_reset(panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "panel_reset: %s", esp_err_to_name(err));
  }

  err = esp_lcd_panel_init(panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "panel_init FAILED: %s", esp_err_to_name(err));
    shutdown(); return false;
  }

  // CRITICAL: Invert OE output via GPIO matrix AFTER panel_init
  invert_oe_output();

  transfer_started_ = true;
  ESP_LOGI(TAG, "LCD DMA running: h_res=%d v_res=%d shift_col=%zu "
           "blanking_comp=%d fb=%.1fKB",
           h_res_, v_res_, shift_col_, blanking_clocks_before_latch_,
           fb_size_ / 1024.0f);

  return true;
}

void LcdDma::shutdown() {
  if (panel_handle_) {
    esp_lcd_panel_del(panel_handle_);
    panel_handle_ = nullptr;
  }
  lcd_fb_[0] = nullptr;
  lcd_fb_[1] = nullptr;
  transfer_started_ = false;
}

// ============================================================================
// OE Inversion — IDF-version-safe via raw register access
// ============================================================================

void LcdDma::invert_oe_output() {
  // The GPIO matrix func_out_sel_cfg register layout:
  //   bits [8:0]  = func_out_sel (peripheral signal index)
  //   bit  [9]    = output inversion (the bit we need to set)
  //   bit  [10]   = oe_sel
  //   bit  [11]   = oe_inv_sel
  //
  // We use raw register access because the struct field name varies
  // between ESP-IDF versions (out_inv_sel / inv_sel / func_n_out_inv_sel).
  GPIO.func_out_sel_cfg[config_.pins.oe].out_inv_sel = 1;
  return;
  volatile uint32_t *reg = &GPIO.func_out_sel_cfg[config_.pins.oe].val;
  *reg |= (1u << 9);  // Set output inversion bit

  // Verify
  if (*reg & (1u << 9)) {
    ESP_LOGI(TAG, "OE GPIO%d inverted via GPIO matrix ✓", config_.pins.oe);
  } else {
    ESP_LOGE(TAG, "OE GPIO%d inversion FAILED — display will be wrong!",
             config_.pins.oe);
  }
}

// ============================================================================
// GPIO
// ============================================================================

void LcdDma::configure_gpio() {
  const gpio_num_t pins[] = {
      (gpio_num_t)config_.pins.r1,  (gpio_num_t)config_.pins.g1,
      (gpio_num_t)config_.pins.b1,  (gpio_num_t)config_.pins.r2,
      (gpio_num_t)config_.pins.g2,  (gpio_num_t)config_.pins.b2,
      (gpio_num_t)config_.pins.a,   (gpio_num_t)config_.pins.b,
      (gpio_num_t)config_.pins.c,   (gpio_num_t)config_.pins.d,
      (gpio_num_t)config_.pins.e,   (gpio_num_t)config_.pins.lat,
      (gpio_num_t)config_.pins.oe,  (gpio_num_t)config_.pins.clk,
  };
  for (auto pin : pins) {
    if (pin >= 0) gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
  }
}

// ============================================================================
// LCD Panel Creation
// ============================================================================

bool LcdDma::create_lcd_panel() {
  ESP_LOGI(TAG, "Creating LCD panel: %d×%d @ %u Hz", h_res_, v_res_, actual_clock_hz_);

  esp_lcd_rgb_panel_config_t cfg{};
  cfg.clk_src         = LCD_CLK_SRC_DEFAULT;
  cfg.timings.pclk_hz = actual_clock_hz_;
  cfg.timings.h_res   = h_res_;
  cfg.timings.v_res   = v_res_;

  // ── HSYNC = LAT timing ──
  //
  // We need exactly 1 PCLK rising edge while HSYNC is HIGH so the panel
  // gets the CLK+LAT combination required by FM6126A and similar chips.
  //
  // Strategy: pclk_idle_high=0 means PCLK idles LOW during blanking.
  // The LCD peripheral still counts internal clock cycles for porch/sync
  // timing, but the PCLK output transitions from the active toggling
  // state to idle LOW, then back to toggling for the next line.
  //
  // With hsync_front_porch=1:
  //   After last active pixel → 1 internal cycle (FP, PCLK transition
  //   to idle LOW) → HSYNC goes HIGH for hsync_pulse_width=1 cycle.
  //
  // During the HSYNC cycle the peripheral generates 1 PCLK edge
  // (LOW→HIGH→LOW or just the edge depending on exact implementation).
  // This provides the CLK+LAT edge. Data=0 is clocked in (1 zero).
  //
  // After HSYNC: back_porch=1 cycle (PCLK idle, doesn't matter — latch
  // already happened).
  //
  // Total extra clocks seen by the shift register between last active
  // pixel and latch = front_porch clocks + HSYNC clock = 1.
  // (FP clock happens with HSYNC still LOW so it only shifts, doesn't
  //  latch. The HSYNC clock is the one that latches.)
  //
  // Actually: with pclk_idle_high=0 the PCLK output is GATED during
  // blanking on ESP32-P4 — it stays LOW, no toggling.  The only edge
  // the shift register sees is the very first rising edge when the next
  // line starts.  But HSYNC has already gone LOW by then.
  //
  // ── Alternative: Rely on level-triggered LAT ──
  //
  // Most HUB75 shift drivers (FM6126A, ICN2038S, MBI5124, 74HC595)
  // use a LEVEL-triggered output latch during normal data operation:
  //   LAT HIGH → output register follows shift register (transparent)
  //   LAT LOW  → output register frozen (latched)
  //
  // The CLK+LAT requirement is only for FM6126A CONFIGURATION mode
  // (handled by driver_init.cpp, not during normal refresh).
  //
  // Therefore: pclk_idle_high=1 (no PCLK during blanking) is safe
  // for all panels during normal refresh. HSYNC goes HIGH for 1 cycle,
  // the output register becomes transparent (captures SR contents),
  // then HSYNC goes LOW and the output register latches.
  //
  // This is the approach we use — simpler and avoids shift compensation.

  cfg.timings.hsync_pulse_width = 1;  // 1 PCLK LAT pulse
  cfg.timings.hsync_back_porch  = 1;
  cfg.timings.hsync_front_porch = 1;

  cfg.timings.vsync_pulse_width = 1;
  cfg.timings.vsync_back_porch  = 0;
  cfg.timings.vsync_front_porch = 0;

  // pclk_idle_high=1: PCLK held HIGH during blanking → no spurious
  // shift register clocks. Level-triggered LAT works without CLK edges.
  cfg.timings.flags.pclk_idle_high  = 1;
  cfg.timings.flags.pclk_active_neg = config_.clk_phase_inverted ? 0 : 1;
  cfg.timings.flags.hsync_idle_low  = 1;  // HSYNC(LAT) idles LOW ✓
  cfg.timings.flags.vsync_idle_low  = 1;
  cfg.timings.flags.de_idle_high    = 1;

  // No blanking compensation needed with pclk_idle_high=1
  blanking_clocks_before_latch_ = 0;

  cfg.data_width     = 16;
  cfg.bits_per_pixel = 16;
  cfg.num_fbs        = config_.double_buffer ? 2 : 1;
  cfg.bounce_buffer_size_px = 0;

  cfg.hsync_gpio_num = (gpio_num_t)config_.pins.lat;
  cfg.vsync_gpio_num = GPIO_NUM_NC;
  cfg.de_gpio_num    = GPIO_NUM_NC;
  cfg.pclk_gpio_num  = (gpio_num_t)config_.pins.clk;
  cfg.disp_gpio_num  = GPIO_NUM_NC;

  std::memset(cfg.data_gpio_nums, -1, sizeof(cfg.data_gpio_nums));
  cfg.data_gpio_nums[PIN_A]     = (gpio_num_t)config_.pins.a;
  cfg.data_gpio_nums[PIN_B]     = (gpio_num_t)config_.pins.b;
  cfg.data_gpio_nums[PIN_C]     = (gpio_num_t)config_.pins.c;
  cfg.data_gpio_nums[PIN_D]     = (gpio_num_t)config_.pins.d;
  cfg.data_gpio_nums[PIN_E]     = (gpio_num_t)config_.pins.e;
  cfg.data_gpio_nums[PIN_OE]    = (gpio_num_t)config_.pins.oe;
  cfg.data_gpio_nums[PIN_P1_R1] = (gpio_num_t)config_.pins.r1;
  cfg.data_gpio_nums[PIN_P1_G1] = (gpio_num_t)config_.pins.g1;
  cfg.data_gpio_nums[PIN_P1_B1] = (gpio_num_t)config_.pins.b1;
  cfg.data_gpio_nums[PIN_P1_R2] = (gpio_num_t)config_.pins.r2;
  cfg.data_gpio_nums[PIN_P1_G2] = (gpio_num_t)config_.pins.g2;
  cfg.data_gpio_nums[PIN_P1_B2] = (gpio_num_t)config_.pins.b2;

#if HUB75_EXTERNAL_FRAMEBUFFERS == 1
  cfg.flags.fb_in_psram = 1;
#else
  cfg.flags.fb_in_psram = 0;
#endif
  cfg.flags.double_fb         = config_.double_buffer ? 1u : 0u;
  cfg.flags.no_fb             = 0;
  cfg.flags.refresh_on_demand = 0;
  cfg.flags.bb_invalidate_cache = 0;

  esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_lcd_new_rgb_panel FAILED: %s (h_res=%d, v_res=%d)",
             esp_err_to_name(err), h_res_, v_res_);
    return false;
  }

  ESP_LOGI(TAG, "LCD panel created (HSYNC=GPIO%d→LAT, PCLK=GPIO%d)",
           config_.pins.lat, config_.pins.clk);
  return true;
}

// ============================================================================
// BCM Timing Calculation
// ============================================================================

size_t LcdDma::calculate_bcm_padding(uint8_t bit_plane) const {
  const size_t base_padding = config_.latch_blanking;
  const size_t base_display = dma_width_ - base_padding;
  if (bit_plane <= lsbMsbTransitionBit_) {
    return base_padding + base_display;
  }
  const size_t reps = 1u << (bit_plane - lsbMsbTransitionBit_ - 1);
  return base_padding + reps * base_display;
}

void LcdDma::calculate_bcm_timings() {
  const uint32_t target_hz = config_.min_refresh_rate;
  static constexpr uint16_t HSYNC_OVERHEAD = 3;  // FP+HSYNC+BP
  static constexpr uint16_t VSYNC_OVERHEAD = 1;

  // The shift section width = dma_width (no blanking compensation
  // needed with pclk_idle_high=1)
  const uint16_t shift_width = dma_width_;

  lsbMsbTransitionBit_ = 0;
  int best_hz = 0;

  while (true) {
    size_t max_line = 0;
    for (int b = 0; b < bit_depth_; b++) {
      size_t total = shift_width + calculate_bcm_padding(b);
      max_line = std::max(max_line, total);
    }

    // ── Enforce hardware limits ──
    // Round up to even (safe for DMA with 16-bit pixels)
    if (max_line & 1) max_line++;
    // Clamp to 12-bit register maximum
    if (max_line > LCD_MAX_H_RES) {
      ESP_LOGD(TAG, "transition=%d → h_res=%zu exceeds %d, increasing",
               lsbMsbTransitionBit_, max_line, LCD_MAX_H_RES);
      if (lsbMsbTransitionBit_ < bit_depth_ - 1) {
        lsbMsbTransitionBit_++;
        continue;
      } else {
        max_line = LCD_MAX_H_RES;
        ESP_LOGW(TAG, "h_res clamped to %d (max hardware limit)", LCD_MAX_H_RES);
      }
    }

    const uint32_t clocks_per_line = max_line + HSYNC_OVERHEAD;
    const uint32_t total_lines = static_cast<uint32_t>(num_rows_) * bit_depth_
                                 + VSYNC_OVERHEAD;
    const float frame_time_s = static_cast<float>(clocks_per_line * total_lines)
                               / static_cast<float>(actual_clock_hz_);
    best_hz = static_cast<int>(1.0f / frame_time_s);

    ESP_LOGD(TAG, "transition=%d → h_res=%zu refresh=%dHz (target %lu)",
             lsbMsbTransitionBit_, max_line, best_hz, (unsigned long)target_hz);

    if (best_hz >= static_cast<int>(target_hz)) break;

    if (lsbMsbTransitionBit_ < bit_depth_ - 1) {
      lsbMsbTransitionBit_++;
    } else {
      ESP_LOGW(TAG, "Cannot reach %luHz, max %dHz",
               (unsigned long)target_hz, best_hz);
      break;
    }
  }

  // Commit final values
  size_t max_line = 0;
  for (int b = 0; b < bit_depth_; b++) {
    bcm_padding_[b] = calculate_bcm_padding(b);
    size_t total = shift_width + bcm_padding_[b];
    max_line = std::max(max_line, total);
  }
  // Round up to even
  if (max_line & 1) max_line++;
  // Clamp (should already be within limits from loop above)
  max_line = std::min(max_line, static_cast<size_t>(LCD_MAX_H_RES));

  h_res_       = static_cast<uint16_t>(max_line);
  v_res_       = num_rows_ * bit_depth_;
  line_stride_ = static_cast<size_t>(h_res_) * 2;
  fb_size_     = line_stride_ * v_res_;
  shift_col_   = h_res_ - shift_width;

  ESP_LOGI(TAG, "BCM: transition=%d h_res=%d v_res=%d shift_col=%zu "
           "fb=%.1fKB refresh≈%dHz",
           lsbMsbTransitionBit_, h_res_, v_res_, shift_col_,
           fb_size_ / 1024.0f, best_hz);
}

// ============================================================================
// Buffer Initialization
// ============================================================================

void LcdDma::initialize_blank_buffer(uint8_t *fb) {
  if (!fb) return;
  std::memset(fb, 0, fb_size_);

  for (int row = 0; row < num_rows_; row++) {
    const uint8_t curr_addr = row & ADDR_MASK;

    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);

      const int prev_bit = (bit == 0) ? (bit_depth_ - 1) : (bit - 1);
      const int prev_row = (bit == 0)
                               ? ((row == 0) ? (num_rows_ - 1) : (row - 1))
                               : row;
      const uint8_t prev_addr = prev_row & ADDR_MASK;

      const size_t padding = bcm_padding_[prev_bit];
      const size_t guard_len = std::min(
          static_cast<size_t>(config_.latch_blanking), padding);
      const size_t main_pad = padding - guard_len;

      size_t col = 0;

      // Padding main: addr = prev_addr, OE_INV=0
      for (size_t i = 0; i < main_pad; i++, col++) {
        pixel_at(fb, line, col)[0] = prev_addr;
      }

      // Padding guard: addr transitions to curr_addr
      for (size_t i = 0; i < guard_len; i++, col++) {
        pixel_at(fb, line, col)[0] = curr_addr;
      }

      // Fill: blanked filler to reach shift_col_
      for (; col < shift_col_; col++) {
        pixel_at(fb, line, col)[0] = curr_addr;
      }

      // Shift: dma_width pixels (pixel data written by draw_pixels)
      for (size_t i = 0; i < dma_width_; i++, col++) {
        pixel_at(fb, line, col)[0] = curr_addr;
      }

      assert(col == h_res_);
    }
  }

  ESP_LOGI(TAG, "Blank buffer initialized (%zu bytes)", fb_size_);
}

// ============================================================================
// Brightness / OE
// ============================================================================

void LcdDma::set_brightness_oe_buffer(uint8_t *fb, uint8_t brightness) {
  if (!fb) return;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);
      const int prev_bit = (bit == 0) ? (bit_depth_ - 1) : (bit - 1);
      const size_t padding = bcm_padding_[prev_bit];

      // Reset all padding OE_INV to 0 (blanked)
      for (size_t i = 0; i < padding; i++) {
        pixel_at(fb, line, i)[0] &= ~OE_INV_MASK;
      }

      if (brightness == 0 || padding == 0) continue;

      const size_t guard = std::min(
          static_cast<size_t>(config_.latch_blanking), padding);
      const int avail = static_cast<int>(padding - guard);
      if (avail < 2) continue;

      int max_display;
      if (prev_bit <= lsbMsbTransitionBit_) {
        const int bitplane   = bit_depth_ - 1 - prev_bit;
        const int bitshift   = (bit_depth_ - lsbMsbTransitionBit_ - 1) >> 1;
        const int rightshift = std::max(bitplane - bitshift - 2, 0);
        max_display = avail >> rightshift;
      } else {
        max_display = avail;
      }
      if (max_display < 2) continue;

      const int eff = remap_brightness(brightness);
      int display_count = (max_display * eff) >> 8;

      const int min_bit = std::max(0, bit_depth_ - 1 - (eff >> 4));
      if (eff > 0 && display_count == 0 && prev_bit >= min_bit) {
        display_count = 1;
      }
      display_count = std::min(display_count, max_display - 1);

      size_t natural_start = (avail - display_count) / 2;
      size_t start = std::max(natural_start, ADDR_SETTLE_PIXELS);
      if (start + display_count > static_cast<size_t>(avail)) {
        display_count = avail - static_cast<int>(start);
      }
      if (display_count <= 0) continue;

      const size_t end = start + display_count;
      for (size_t i = start; i < end; i++) {
        pixel_at(fb, line, i)[0] |= OE_INV_MASK;
      }
    }
  }
}

void LcdDma::set_brightness_oe() {
  const uint8_t bri = static_cast<uint8_t>(
      static_cast<float>(basis_brightness_) * intensity_);
  for (auto fb : lcd_fb_) {
    if (fb) set_brightness_oe_buffer(fb, bri);
  }
  flush_cache();
}

void LcdDma::flush_cache() {
#if HUB75_EXTERNAL_FRAMEBUFFERS == 1
  uint8_t *fb = lcd_fb_[active_idx_];
  if (fb && esp_ptr_external_ram(fb)) {
    esp_cache_msync(fb, fb_size_,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  }
#endif
}

// ============================================================================
// Pixel Drawing
// ============================================================================

HUB75_IRAM void LcdDma::draw_pixels(uint16_t x, uint16_t y, uint16_t w,
                                     uint16_t h, const uint8_t *buffer,
                                     Hub75PixelFormat format,
                                     Hub75ColorOrder color_order,
                                     bool big_endian) {
  uint8_t *fb = lcd_fb_[active_idx_];
  if (!fb || !buffer) [[unlikely]] return;

  const uint16_t rot_w = RotationTransform::get_rotated_width(
      virtual_width_, virtual_height_, rotation_);
  const uint16_t rot_h = RotationTransform::get_rotated_height(
      virtual_width_, virtual_height_, rotation_);

  if (x >= rot_w || y >= rot_h) [[unlikely]] return;
  if (x + w > rot_w) w = rot_w - x;
  if (y + h > rot_h) h = rot_h - y;

  const size_t stride = (format == Hub75PixelFormat::RGB888)   ? 3
                      : (format == Hub75PixelFormat::RGB565)   ? 2
                      : /* RGB888_32 */ 4;

  const bool identity = (rotation_ == Hub75Rotation::ROTATE_0)
                        && !needs_layout_remap_ && !needs_scan_remap_;

  const uint8_t *ptr = buffer;

  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx, py = y + dy;
      uint16_t row; bool is_lower;

      if (identity) {
        if (py < num_rows_) { row = py; is_lower = false; }
        else { row = py - num_rows_; is_lower = true; }
      } else {
        auto t = transform_coordinate(
            px, py, rotation_, needs_layout_remap_, needs_scan_remap_,
            layout_, scan_wiring_, panel_width_, panel_height_,
            layout_rows_, layout_cols_, virtual_width_, virtual_height_,
            dma_width_, num_rows_);
        px = t.x; row = t.row; is_lower = t.is_lower;
      }

      uint8_t r8, g8, b8;
      extract_rgb888_from_format(ptr, 0, format, color_order, big_endian,
                                 r8, g8, b8);
      ptr += stride;

      const uint16_t rc = lut_[r8];
      const uint16_t gc = lut_[g8];
      const uint16_t bc = lut_[b8];

      const uint8_t clr = is_lower ? P1_LOWER_MASK : P1_UPPER_MASK;
      const int base = row * bit_depth_;

      for (int bit = 0; bit < bit_depth_; bit++) {
        const uint8_t rb = (rc >> bit) & 1;
        const uint8_t gb = (gc >> bit) & 1;
        const uint8_t bb = (bc >> bit) & 1;

        uint8_t rgb;
        if (is_lower)
          rgb = (rb << P1_R2) | (gb << P1_G2) | (bb << P1_B2);
        else
          rgb = (rb << P1_R1) | (gb << P1_G1) | (bb << P1_B1);

        uint8_t &byte1 = pixel_at(fb, base + bit, shift_col_ + px)[1];
        byte1 = (byte1 & ~clr) | rgb;
      }
    }
  }

  if (!is_double_buffered_) flush_cache();
}

// ============================================================================
// Clear / Fill
// ============================================================================

void LcdDma::clear() {
  uint8_t *fb = lcd_fb_[active_idx_];
  if (!fb) return;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);
      for (int col = 0; col < dma_width_; col++) {
        pixel_at(fb, line, shift_col_ + col)[1] &= ~P1_RGB_MASK;
      }
    }
  }
  if (!is_double_buffered_) flush_cache();
}

HUB75_IRAM void LcdDma::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              uint8_t r, uint8_t g, uint8_t b) {
  uint8_t *fb = lcd_fb_[active_idx_];
  if (!fb) [[unlikely]] return;

  const uint16_t rot_w = RotationTransform::get_rotated_width(
      virtual_width_, virtual_height_, rotation_);
  const uint16_t rot_h = RotationTransform::get_rotated_height(
      virtual_width_, virtual_height_, rotation_);

  if (x >= rot_w || y >= rot_h) [[unlikely]] return;
  if (x + w > rot_w) w = rot_w - x;
  if (y + h > rot_h) h = rot_h - y;

  const uint16_t rc = lut_[r];
  const uint16_t gc = lut_[g];
  const uint16_t bc = lut_[b];

  uint8_t upper_pat[HUB75_MAX_BIT_DEPTH];
  uint8_t lower_pat[HUB75_MAX_BIT_DEPTH];
  for (int bit = 0; bit < bit_depth_; bit++) {
    const uint8_t rb = (rc >> bit) & 1;
    const uint8_t gb = (gc >> bit) & 1;
    const uint8_t bb = (bc >> bit) & 1;
    upper_pat[bit] = (rb << P1_R1) | (gb << P1_G1) | (bb << P1_B1);
    lower_pat[bit] = (rb << P1_R2) | (gb << P1_G2) | (bb << P1_B2);
  }

  const bool identity = (rotation_ == Hub75Rotation::ROTATE_0)
                        && !needs_layout_remap_ && !needs_scan_remap_;

  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx, py = y + dy;
      uint16_t row; bool is_lower;

      if (identity) {
        if (py < num_rows_) { row = py; is_lower = false; }
        else { row = py - num_rows_; is_lower = true; }
      } else {
        auto t = transform_coordinate(
            px, py, rotation_, needs_layout_remap_, needs_scan_remap_,
            layout_, scan_wiring_, panel_width_, panel_height_,
            layout_rows_, layout_cols_, virtual_width_, virtual_height_,
            dma_width_, num_rows_);
        px = t.x; row = t.row; is_lower = t.is_lower;
      }

      const uint8_t clr  = is_lower ? P1_LOWER_MASK : P1_UPPER_MASK;
      const uint8_t *pat = is_lower ? lower_pat : upper_pat;
      const int base_line = row * bit_depth_;

      for (int bit = 0; bit < bit_depth_; bit++) {
        uint8_t &byte1 = pixel_at(fb, base_line + bit, shift_col_ + px)[1];
        byte1 = (byte1 & ~clr) | pat[bit];
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

  esp_err_t err = esp_lcd_panel_draw_bitmap(
      panel_handle_, 0, 0, h_res_, v_res_, lcd_fb_[active_idx_]);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "flip: %s", esp_err_to_name(err));
  }
  std::swap(front_idx_, active_idx_);
}

// ============================================================================
// Transfer Control
// ============================================================================

void LcdDma::start_transfer()  { transfer_started_ = true; }
void LcdDma::stop_transfer()   { transfer_started_ = false; }

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

void LcdDma::set_rotation(Hub75Rotation rotation) {
  rotation_ = rotation;
}

}  // namespace hub75

#endif