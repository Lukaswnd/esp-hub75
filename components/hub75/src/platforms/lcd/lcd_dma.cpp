// SPDX-License-Identifier: MIT
//
// @file lcd_dma.cpp
// @brief LCD RGB peripheral HUB75 implementation for ESP32-P4
//
// ┌─────────────────────────────────────────────────────────────────┐
// │  HSYNC = LAT  approach                                         │
// │                                                                │
// │  The LCD peripheral's HSYNC signal is connected to the HUB75   │
// │  LAT (latch) pin. This frees a data pin, enabling up to 3     │
// │  independent panel chains on the 24-bit data bus:              │
// │                                                                │
// │    5 (A-E) + 1 (OE) + 3×6 (RGB) = 24 data pins               │
// │    + HSYNC→LAT  + PCLK→CLK      = 26 total GPIOs              │
// │                                                                │
// │  OE is inverted via the GPIO matrix after LCD panel init so    │
// │  that data=0 → GPIO HIGH → HUB75 blanked. This is critical    │
// │  because the LCD peripheral drives all data to 0 during        │
// │  HSYNC/VSYNC blanking, which must blank the display.           │
// │                                                                │
// │  Each LCD line = one (row, bit_plane) pair, structured as:     │
// │                                                                │
// │  ┌──────────┬──────────┬──────────────────────────┐            │
// │  │ Padding  │   Fill   │  Shift (dma_width pixels) │─→HSYNC    │
// │  │ (BCM     │ (blanked │  (blanked, pixel data)    │  =LAT     │
// │  │ display) │  filler) │                            │           │
// │  └──────────┴──────────┴──────────────────────────┘            │
// │  ↑ addr=prev_row       ↑ addr=curr_row             ↑           │
// │  OE controlled         OE_INV=0 (off)              shift_col_  │
// │  by brightness                                                 │
// │                                                                │
// │  Shift section is always the LAST dma_width pixels of each     │
// │  line, at a fixed column offset (shift_col_). This ensures     │
// │  the shift register contains exactly the pixel data when       │
// │  HSYNC/LAT fires.                                              │
// │                                                                │
// │  pclk_idle_high=1 prevents spurious shift register clocks      │
// │  during HSYNC/VSYNC blanking.                                  │
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
// 24-bit word layout constants
// ============================================================================
//
// HSYNC = LAT (not on data bus). OE is inverted via GPIO matrix.
//
// Byte 0 [GPIO 0–7]:   A(0) B(1) C(2) D(3) E(4) OE_INV(5) --(6) --(7)
// Byte 1 [GPIO 8–15]:  P1_R1(0) P1_G1(1) P1_B1(2) P1_R2(3) P1_G2(4) P1_B2(5) --(6) --(7)
// Byte 2 [GPIO 16–23]: P2_R1(0) P2_G1(1) P2_B1(2) P2_R2(3) P2_G2(4) P2_B2(5) --(6) --(7)
//
// OE_INV semantics (after GPIO inversion):
//   bit=0 → GPIO HIGH → HUB75 OE=HIGH → BLANKED (display off)
//   bit=1 → GPIO LOW  → HUB75 OE=LOW  → DISPLAY ON

// Byte 0: control
static constexpr uint8_t ADDR_MASK    = 0x1F;     // Bits 0–4: 5-bit row address
static constexpr uint8_t OE_INV_BIT   = 5;
static constexpr uint8_t OE_INV_MASK  = 1u << OE_INV_BIT;

// Byte 1: Panel 1 RGB (bit positions relative to byte 1)
static constexpr uint8_t P1_R1 = 0, P1_G1 = 1, P1_B1 = 2;
static constexpr uint8_t P1_R2 = 3, P1_G2 = 4, P1_B2 = 5;
static constexpr uint8_t P1_UPPER_MASK = (1u << P1_R1) | (1u << P1_G1) | (1u << P1_B1);  // 0x07
static constexpr uint8_t P1_LOWER_MASK = (1u << P1_R2) | (1u << P1_G2) | (1u << P1_B2);  // 0x38
static constexpr uint8_t P1_RGB_MASK   = P1_UPPER_MASK | P1_LOWER_MASK;                   // 0x3F

// GPIO data bus indices (position in 16-bit LCD word)
static constexpr int PIN_A      =  0;
static constexpr int PIN_B      =  1;
static constexpr int PIN_C      =  2;
static constexpr int PIN_D      =  3;
static constexpr int PIN_E      =  4;
static constexpr int PIN_OE     =  5;
// 6, 7: unused (available for 3-panel tight packing)
static constexpr int PIN_P1_R1  =  8;
static constexpr int PIN_P1_G1  =  9;
static constexpr int PIN_P1_B1  = 10;
static constexpr int PIN_P1_R2  = 11;
static constexpr int PIN_P1_G2  = 12;
static constexpr int PIN_P1_B2  = 13;
// 14–23: reserved for panel 2 / 3
// Tight 3-panel packing (future): move P1_R1/G1 into byte 0 bits 6–7

// Minimum blanking at start of display window for address settling after row change
static constexpr size_t ADDR_SETTLE_PIXELS = 2;

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
      h_res_(0),
      v_res_(0),
      line_stride_(0),
      fb_size_(0),
      shift_col_(0),
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
  ESP_LOGI(TAG, "=== LCD RGB DMA for HUB75 (HSYNC=LAT, 16-bit bus) ===");
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d",
           panel_width_, panel_height_, layout_cols_, layout_rows_,
           virtual_width_, virtual_height_);
  ESP_LOGI(TAG, "DMA: width=%d, rows=%d, bit_depth=%d, four_scan=%s",
           dma_width_, num_rows_, bit_depth_,
           is_four_scan_wiring(scan_wiring_) ? "yes" : "no");

  // 1. Calculate BCM timings → sets h_res_, v_res_, lsbMsbTransitionBit_, shift_col_
  calculate_bcm_timings();

  // 2. Brightness remapping coefficients (quadratic curve)
  init_brightness_coeffs(dma_width_, config_.latch_blanking);

  // 3. Adjust LUT for BCM monotonicity when transition bit > 0
#if HUB75_GAMMA_MODE == 1 || HUB75_GAMMA_MODE == 2
  if (lsbMsbTransitionBit_ > 0) {
    int adj = adjust_lut_for_bcm(lut_, bit_depth_, lsbMsbTransitionBit_);
    ESP_LOGI(TAG, "Adjusted %d LUT entries for BCM (transition=%d)", adj, lsbMsbTransitionBit_);
  }
#endif

  // 4. GPIO drive strength
  configure_gpio();

  // 5. Create LCD panel (allocates PSRAM framebuffers, does NOT start DMA)
  if (!create_lcd_panel()) {
    ESP_LOGE(TAG, "Failed to create LCD panel");
    return false;
  }

  // 6. Obtain framebuffer pointers from LCD driver
  if (config_.double_buffer) {
    void *fb0 = nullptr, *fb1 = nullptr;
    esp_err_t err = esp_lcd_rgb_panel_get_frame_buffer(panel_handle_, 2, &fb0, &fb1);
    if (err != ESP_OK || !fb0) {
      ESP_LOGE(TAG, "get_frame_buffer(2) failed: %s", esp_err_to_name(err));
      shutdown();
      return false;
    }
    lcd_fb_[0] = static_cast<uint8_t *>(fb0);
    lcd_fb_[1] = static_cast<uint8_t *>(fb1);
    is_double_buffered_ = (lcd_fb_[1] != nullptr);
  } else {
    void *fb0 = nullptr;
    esp_err_t err = esp_lcd_rgb_panel_get_frame_buffer(panel_handle_, 1, &fb0);
    if (err != ESP_OK || !fb0) {
      ESP_LOGE(TAG, "get_frame_buffer(1) failed: %s", esp_err_to_name(err));
      shutdown();
      return false;
    }
    lcd_fb_[0] = static_cast<uint8_t *>(fb0);
  }

  front_idx_  = 0;
  active_idx_ = is_double_buffered_ ? 1 : 0;

  ESP_LOGI(TAG, "Framebuffers: fb[0]=%p, fb[1]=%p (%s)",
           lcd_fb_[0], lcd_fb_[1],
           is_double_buffered_ ? "double" : "single");

  // 7. Build BCM structure (address bits, OE_INV=0 everywhere → blanked, RGB=0)
  initialize_blank_buffer(lcd_fb_[0]);
  if (lcd_fb_[1]) {
    initialize_blank_buffer(lcd_fb_[1]);
  }

  // 8. Apply brightness-dependent OE pattern in padding sections
  set_brightness_oe();

  // 9. Flush CPU cache → PSRAM before DMA starts reading
  for (auto fb : lcd_fb_) {
    if (fb && esp_ptr_external_ram(fb)) {
      esp_cache_msync(fb, fb_size_,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
  }

  // 10. Pre-set OE pin HIGH (blanked) before LCD takes over GPIO control.
  //     This minimises the glitch window between panel_init() (which
  //     reconfigures the pin as LCD data output) and our inversion below.
  gpio_set_direction((gpio_num_t)config_.pins.oe, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)config_.pins.oe, 1);  // HUB75 OE=HIGH → blanked

  // 11. Reset & init LCD → starts continuous DMA refresh
  esp_err_t err = esp_lcd_panel_reset(panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "panel_reset: %s (non-fatal)", esp_err_to_name(err));
  }

  err = esp_lcd_panel_init(panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "panel_init failed: %s", esp_err_to_name(err));
    shutdown();
    return false;
  }

  // 12. CRITICAL: Invert OE output via GPIO matrix IMMEDIATELY after init.
  //
  //     esp_lcd_panel_init() just configured PIN_OE as an LCD data output.
  //     Without inversion: OE_INV=0 in framebuffer → GPIO LOW → HUB75 OE=LOW
  //       → display ON (shows garbage during shift phase!)
  //     With inversion:    OE_INV=0 → GPIO HIGH → HUB75 OE=HIGH → blanked ✓
  //
  //     During HSYNC/VSYNC blanking the LCD peripheral drives all data to 0.
  //     With inversion: data=0 → GPIO HIGH → blanked. Exactly what we need.
  {
    GPIO.func_out_sel_cfg[config_.pins.oe].out_inv_sel = 1;
    ESP_LOGI(TAG, "OE GPIO %d invertiert via GPIO-Matrix", config_.pins.oe);
  }

  transfer_started_ = true;

  // Log summary
  const float frame_time_ms =
      static_cast<float>(h_res_ + 1 + 1 + 1)  // h_res + front_porch + hsync + back_porch
      * static_cast<float>(v_res_ + 1)          // v_res + vsync

      / actual_clock_hz_ * 1000.0f;
  ESP_LOGI(TAG, "LCD DMA running: H_RES=%d, V_RES=%d, shift_col=%zu, "
           "%.1f KB/fb, ~%.0f Hz",
           h_res_, v_res_, shift_col_, fb_size_ / 1024.0f,
           1000.0f / frame_time_ms);

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
  ESP_LOGI(TAG, "Shutdown complete");
}

// ============================================================================
// GPIO
// ============================================================================

void LcdDma::configure_gpio() {
  // Note: LAT is configured as HSYNC (not in this list).
  //       OE inversion happens after panel init, not here.
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
    if (pin >= 0) {
      gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
    }
  }
}

// ============================================================================
// LCD Panel Creation
// ============================================================================

bool LcdDma::create_lcd_panel() {
  ESP_LOGI(TAG, "Creating LCD RGB panel: %d×%d @ %u Hz (HSYNC=LAT)",
           h_res_, v_res_, actual_clock_hz_);

  esp_lcd_rgb_panel_config_t cfg{};

  cfg.clk_src             = LCD_CLK_SRC_DEFAULT;
  cfg.timings.pclk_hz     = actual_clock_hz_;
  cfg.timings.h_res       = h_res_;
  cfg.timings.v_res       = v_res_;

  // HSYNC = LAT: one PCLK-cycle pulse, minimal porches.
  // The HSYNC pulse latches the shift register contents into the panel's
  // output register. Front/back porch of 1 avoids corner cases where the
  // peripheral requires non-zero porch values.
  cfg.timings.hsync_pulse_width  = 1;   // 1-PCLK LAT pulse
  cfg.timings.hsync_back_porch   = 1;   // After HSYNC, before next line data
  cfg.timings.hsync_front_porch  = 1;   // After last pixel, before HSYNC

  // VSYNC: minimal (not connected, but required by driver)
  cfg.timings.vsync_pulse_width  = 1;
  cfg.timings.vsync_back_porch   = 0;
  cfg.timings.vsync_front_porch  = 0;

  // Clock configuration:
  //   pclk_idle_high = 1  → PCLK stays HIGH during blanking (HSYNC/VSYNC)
  //     → no rising edges → no spurious shift register clocks
  //   pclk_active_neg = 1 → data driven on falling edge
  //     → HUB75 panel samples on rising edge (maximum setup time)
  //   hsync_idle_low = 1  → HSYNC idles LOW, pulses HIGH
  //     → HUB75 LAT is active HIGH ✓
  cfg.timings.flags.pclk_idle_high  = 1;
  cfg.timings.flags.pclk_active_neg = config_.clk_phase_inverted ? 0 : 1;
  cfg.timings.flags.hsync_idle_low  = 1;
  cfg.timings.flags.vsync_idle_low  = 1;
  cfg.timings.flags.de_idle_high    = 1;

  // 16-bit data bus, 3 bytes per pixel
  cfg.data_width       = 16;
  cfg.bits_per_pixel   = 16;
  cfg.num_fbs          = config_.double_buffer ? 2 : 1;
  cfg.bounce_buffer_size_px = 0;  // ESP32-P4 GDMA accesses PSRAM directly

  // HSYNC → LAT pin.  All other timing signals unused.
  cfg.hsync_gpio_num = (gpio_num_t)config_.pins.lat;
  cfg.vsync_gpio_num = GPIO_NUM_NC;
  cfg.de_gpio_num    = GPIO_NUM_NC;
  cfg.pclk_gpio_num  = (gpio_num_t)config_.pins.clk;
  cfg.disp_gpio_num  = GPIO_NUM_NC;

  // Data pin mapping – LAT is NOT on the data bus (it's HSYNC).
  std::memset(cfg.data_gpio_nums, -1, sizeof(cfg.data_gpio_nums));
  cfg.data_gpio_nums[PIN_A]      = (gpio_num_t)config_.pins.a;
  cfg.data_gpio_nums[PIN_B]      = (gpio_num_t)config_.pins.b;
  cfg.data_gpio_nums[PIN_C]      = (gpio_num_t)config_.pins.c;
  cfg.data_gpio_nums[PIN_D]      = (gpio_num_t)config_.pins.d;
  cfg.data_gpio_nums[PIN_E]      = (gpio_num_t)config_.pins.e;
  cfg.data_gpio_nums[PIN_OE]     = (gpio_num_t)config_.pins.oe;  // Inverted post-init
  cfg.data_gpio_nums[PIN_P1_R1]  = (gpio_num_t)config_.pins.r1;
  cfg.data_gpio_nums[PIN_P1_G1]  = (gpio_num_t)config_.pins.g1;
  cfg.data_gpio_nums[PIN_P1_B1]  = (gpio_num_t)config_.pins.b1;
  cfg.data_gpio_nums[PIN_P1_R2]  = (gpio_num_t)config_.pins.r2;
  cfg.data_gpio_nums[PIN_P1_G2]  = (gpio_num_t)config_.pins.g2;
  cfg.data_gpio_nums[PIN_P1_B2]  = (gpio_num_t)config_.pins.b2;
  // TODO: Pins 14–23 for panel 2 / panel 3

  // Memory
#if HUB75_EXTERNAL_FRAMEBUFFERS == 1
  cfg.flags.fb_in_psram       = 1;
#else
  cfg.flags.fb_in_psram       = 0;
#endif
  cfg.flags.double_fb         = config_.double_buffer ? 1u : 0u;
  cfg.flags.no_fb             = 0;
  cfg.flags.refresh_on_demand = 0;  // Continuous DMA loop
  cfg.flags.bb_invalidate_cache = 0;

  esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &panel_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_lcd_new_rgb_panel: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGI(TAG, "LCD panel created (HSYNC=GPIO%d→LAT, PCLK=GPIO%d, DMA not started)",
           config_.pins.lat, config_.pins.clk);
  return true;
}

// ============================================================================
// BCM Timing Calculation
// ============================================================================

size_t LcdDma::calculate_bcm_padding(uint8_t bit_plane) const {
  // Identical to parlio_dma.cpp:
  //   LSB bits (≤ transition): padding = base_padding + base_display
  //   MSB bits (>  transition): padding = base_padding + reps × base_display
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

  // LCD line overhead: front_porch(1) + hsync(1) + back_porch(1) = 3 clocks
  static constexpr uint16_t HSYNC_OVERHEAD = 3;
  static constexpr uint16_t VSYNC_OVERHEAD = 1;

  lsbMsbTransitionBit_ = 0;
  int best_hz = 0;

  while (true) {
    // Determine max line width across all bit planes
    size_t max_line = 0;
    for (int b = 0; b < bit_depth_; b++) {
      // In HSYNC=LAT layout, each line's padding comes from the PREVIOUS
      // bit plane.  But max(bcm_padding[b] + dma_width) is the same
      // regardless of which line contains which padding, because we
      // always pad to the maximum.
      size_t total = dma_width_ + calculate_bcm_padding(b);
      max_line = std::max(max_line, total);
    }

    const uint32_t clocks_per_line = max_line + HSYNC_OVERHEAD;
    const uint32_t total_lines     = static_cast<uint32_t>(num_rows_) * bit_depth_
                                     + VSYNC_OVERHEAD;

    const float frame_time_s       = static_cast<float>(clocks_per_line * total_lines)
                                     / static_cast<float>(actual_clock_hz_);
    best_hz = static_cast<int>(1.0f / frame_time_s);

    ESP_LOGD(TAG, "transition=%d → h_res=%zu, refresh=%d Hz (target %lu)",
             lsbMsbTransitionBit_, max_line, best_hz, (unsigned long)target_hz);

    if (best_hz >= static_cast<int>(target_hz)) break;

    if (lsbMsbTransitionBit_ < bit_depth_ - 1) {
      lsbMsbTransitionBit_++;
    } else {
      ESP_LOGW(TAG, "Cannot reach %lu Hz, max %d Hz",
               (unsigned long)target_hz, best_hz);
      break;
    }
  }

  // Commit final values
  size_t max_line = 0;
  for (int b = 0; b < bit_depth_; b++) {
    bcm_padding_[b] = calculate_bcm_padding(b);
    size_t total = dma_width_ + bcm_padding_[b];
    max_line = std::max(max_line, total);
  }

  h_res_       = static_cast<uint16_t>(max_line);
  v_res_       = num_rows_ * bit_depth_;
  line_stride_ = static_cast<size_t>(h_res_) * 2;
  fb_size_     = line_stride_ * v_res_;

  // Shift section starts at the same fixed column for ALL lines.
  // The last dma_width_ pixels of each line are always the shift section.
  shift_col_   = h_res_ - dma_width_;

  // Logging
  size_t useful = 0;
  for (int b = 0; b < bit_depth_; b++) {
    useful += (dma_width_ + bcm_padding_[b]) * num_rows_;
  }
  const float waste_pct = (1.0f - static_cast<float>(useful)
                           / (h_res_ * v_res_)) * 100.0f;

  ESP_LOGI(TAG, "BCM: transition=%d, h_res=%d, v_res=%d, shift_col=%zu, "
           "fb=%.1f KB, waste=%.0f%%, refresh≈%d Hz",
           lsbMsbTransitionBit_, h_res_, v_res_, shift_col_,
           fb_size_ / 1024.0f, waste_pct, best_hz);
}

// ============================================================================
// Buffer Initialization
// ============================================================================
//
// Line (row R, bit B) structure within the LCD framebuffer:
//
//   Col 0                              shift_col_         h_res_-1
//   │← Padding ─→│← Fill (blanked) ─→│← Shift (pixel data) ─→│
//
// Padding: Displays data latched by the PREVIOUS line's HSYNC.
//   - First (padding - latch_blanking) pixels: addr = prev_row
//   - Last latch_blanking pixels: addr = curr_row (address settling guard)
//
// Fill: Blanked filler to reach fixed h_res_ width.
//   - addr = curr_row, OE_INV = 0
//
// Shift: Pixel data shifted into the panel's shift register.
//   - addr = curr_row, OE_INV = 0, RGB = pixel data (set by draw_pixels)
//   - MUST be the last dma_width_ pixels so the shift register contains
//     exactly the pixel data when HSYNC (LAT) fires.

void LcdDma::initialize_blank_buffer(uint8_t *fb) {
  if (!fb) return;

  // Zero entire buffer (efficient, ensures all unused bits/bytes = 0)
  std::memset(fb, 0, fb_size_);

  for (int row = 0; row < num_rows_; row++) {
    const uint8_t curr_addr = row & ADDR_MASK;

    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);

      // Previous bit plane (whose data is displayed in this line's padding)
      const int prev_bit = (bit == 0) ? (bit_depth_ - 1) : (bit - 1);
      const int prev_row = (bit == 0)
                               ? ((row == 0) ? (num_rows_ - 1) : (row - 1))
                               : row;
      const uint8_t prev_addr = prev_row & ADDR_MASK;

      // Padding size = BCM weight of the previous bit plane
      const size_t padding = bcm_padding_[prev_bit];

      // Boundary between main padding and address guard
      const size_t guard_len = std::min(static_cast<size_t>(config_.latch_blanking),
                                        padding);
      const size_t main_pad  = padding - guard_len;

      size_t col = 0;

      // --- Padding: main section (display previous latch, addr=prev_row) ---
      // OE_INV = 0 here (blanked). Brightness function sets OE_INV=1 where needed.
      for (size_t i = 0; i < main_pad; i++, col++) {
        pixel_at(fb, line, col)[0] = prev_addr;  // OE_INV=0, RGB=0 (memset)
      }

      // --- Padding: address guard (addr transitions to curr_row, blanked) ---
      for (size_t i = 0; i < guard_len; i++, col++) {
        pixel_at(fb, line, col)[0] = curr_addr;  // OE_INV=0
      }

      // --- Fill: blanked filler to reach shift_col_ ---
      for (; col < shift_col_; col++) {
        pixel_at(fb, line, col)[0] = curr_addr;  // OE_INV=0
      }

      // --- Shift: dma_width_ pixels (pixel data, blanked) ---
      for (size_t i = 0; i < dma_width_; i++, col++) {
        pixel_at(fb, line, col)[0] = curr_addr;  // OE_INV=0
        // byte 1 (panel 1 RGB) = 0 (from memset, set by draw_pixels)
        // byte 2 (panel 2 RGB) = 0 (from memset, future)
      }

      assert(col == h_res_);
    }
  }

  ESP_LOGI(TAG, "Blank buffer initialized (%zu bytes)", fb_size_);
}

// ============================================================================
// Brightness / OE Control
// ============================================================================
//
// Only the PADDING section's OE_INV bits are modified.
// Shift and fill sections always remain OE_INV=0 (blanked).
//
// The display window is centered in the "available" portion of the padding
// (excluding the latch_blanking guard at the end). A minimum start blank
// of ADDR_SETTLE_PIXELS prevents address glitches after row transitions.

void LcdDma::set_brightness_oe_buffer(uint8_t *fb, uint8_t brightness) {
  if (!fb) return;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      const int line = line_index(row, bit);
      const int prev_bit = (bit == 0) ? (bit_depth_ - 1) : (bit - 1);
      const size_t padding = bcm_padding_[prev_bit];

      // --- Step 1: Reset all padding OE_INV to 0 (blanked) ---
      for (size_t i = 0; i < padding; i++) {
        pixel_at(fb, line, i)[0] &= ~OE_INV_MASK;
      }

      if (brightness == 0 || padding == 0) continue;

      // --- Step 2: Calculate available display region ---
      // The last latch_blanking pixels are always blanked (address guard).
      const size_t guard = std::min(static_cast<size_t>(config_.latch_blanking),
                                    padding);
      const int avail = static_cast<int>(padding - guard);
      if (avail < 2) continue;

      // --- Step 3: BCM differentiation (identical to parlio_dma.cpp) ---
      // LSB bits (≤ transition): identical padding, rightshift reduces OE time
      // MSB bits (> transition):  padding SIZE provides BCM, full OE
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

      // --- Step 4: Apply quadratic brightness remapping ---
      const int eff = remap_brightness(brightness);
      int display_count = (max_display * eff) >> 8;

      // Gradual minimum: at low brightness, only MSB bits get display=1
      const int min_bit = std::max(0, bit_depth_ - 1 - (eff >> 4));
      if (eff > 0 && display_count == 0 && prev_bit >= min_bit) {
        display_count = 1;
      }

      // Safety: keep at least 1 pixel blanked
      display_count = std::min(display_count, max_display - 1);

      // --- Step 5: Center display window, enforce address settling ---
      size_t natural_start = (avail - display_count) / 2;
      size_t start = std::max(natural_start, ADDR_SETTLE_PIXELS);

      // Re-clamp if start was pushed forward
      if (start + display_count > static_cast<size_t>(avail)) {
        display_count = avail - static_cast<int>(start);
      }
      if (display_count <= 0) continue;

      const size_t end = start + display_count;

      // --- Step 6: Set OE_INV=1 (display on) in the window ---
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
//
// Pixel data lives in the SHIFT section of each LCD line, which always
// starts at column shift_col_.  Only byte 1 (panel 1 RGB) is modified;
// byte 0 (addr + OE_INV) is set once in initialize_blank_buffer and
// updated only by brightness functions.

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
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      uint16_t row;
      bool is_lower;

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

      // Extract RGB888
      uint8_t r8, g8, b8;
      extract_rgb888_from_format(ptr, 0, format, color_order, big_endian,
                                 r8, g8, b8);
      ptr += stride;

      // Apply gamma/CIE LUT
      const uint16_t rc = lut_[r8];
      const uint16_t gc = lut_[g8];
      const uint16_t bc = lut_[b8];

      // Update all bit planes – only byte 1 (panel 1 RGB)
      const uint8_t clr = is_lower ? P1_LOWER_MASK : P1_UPPER_MASK;
      const int base = row * bit_depth_;

      for (int bit = 0; bit < bit_depth_; bit++) {
        const uint8_t rb = (rc >> bit) & 1;
        const uint8_t gb = (gc >> bit) & 1;
        const uint8_t bb = (bc >> bit) & 1;

        uint8_t rgb;
        if (is_lower) {
          rgb = (rb << P1_R2) | (gb << P1_G2) | (bb << P1_B2);
        } else {
          rgb = (rb << P1_R1) | (gb << P1_G1) | (bb << P1_B1);
        }

        // Pixel at (shift_col_ + px) in the LCD line
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

  // Clear RGB bits (byte 1) in the shift section of every line
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

  // Pre-compute LUT-corrected values
  const uint16_t rc = lut_[r];
  const uint16_t gc = lut_[g];
  const uint16_t bc = lut_[b];

  // Pre-compute RGB byte patterns for each bit plane
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
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      uint16_t row;
      bool is_lower;

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

      const uint8_t clr   = is_lower ? P1_LOWER_MASK : P1_UPPER_MASK;
      const uint8_t *pat  = is_lower ? lower_pat : upper_pat;
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
    ESP_LOGW(TAG, "draw_bitmap (flip): %s", esp_err_to_name(err));
  }

  std::swap(front_idx_, active_idx_);
}

// ============================================================================
// Transfer Control
// ============================================================================

void LcdDma::start_transfer() {
  // LCD continuous refresh starts automatically in init() via esp_lcd_panel_init().
  transfer_started_ = true;
}

void LcdDma::stop_transfer() {
  if (panel_handle_ && transfer_started_) {
    transfer_started_ = false;
  }
}

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

void LcdDma::set_rotation(Hub75Rotation rotation) {
  rotation_ = rotation;
}

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32P4