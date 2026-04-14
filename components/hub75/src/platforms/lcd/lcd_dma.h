// SPDX-License-Identifier: MIT
//
// @file lcd_dma.h
// @brief LCD RGB peripheral implementation for HUB75 (ESP32-P4)
//
// Uses the ESP32-P4 LCD_CAM peripheral in 24-bit RGB mode as a
// generic parallel data output for HUB75 panels. BCM timing is
// embedded in the framebuffer via variable-length padding per bit
// plane, following the proven approach from parlio_dma.cpp.
//
// Advantage over PARLIO (16-bit): 24-bit data bus allows driving
// up to 3 independent HUB75 panel chains simultaneously.

#pragma once

#include <sdkconfig.h>

#include "../platform_detect.h"
#ifdef HUB75_DMA_ENGINE_LCD

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"
#include "../platform_dma.h"

#include <cstddef>
#include <cstdint>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_panel_ops.h>

namespace hub75 {

/**
 * @brief LCD RGB peripheral implementation for HUB75 (ESP32-P4)

 *
 * Framebuffer layout (LCD perspective):
 *   H_RES = max line width across all bit planes (fixed, with fill padding)
 *   V_RES = num_rows × bit_depth

 *
 * Each LCD "line" = one bit-plane for one row:
 *   [Shift: dma_width pixels, OE=HIGH] → [LAT on last pixel]
 *   [Padding: BCM display time, OE controlled] → [Fill to H_RES, OE=HIGH]

 *
 * HSYNC fires between lines but is not connected to any pin.
 * PCLK idles HIGH during blanking → no spurious shift register clocks.
 * LAT is on a data pin for precise control.

 */
class LcdDma : public PlatformDma {
 public:
  explicit LcdDma(const Hub75Config &config);
  ~LcdDma() override;

  bool init() override;
  void shutdown() override;
  void start_transfer() override;
  void stop_transfer() override;
  void set_basis_brightness(uint8_t brightness) override;
  void set_intensity(float intensity) override;
  void set_rotation(Hub75Rotation rotation) override;

  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian) override;
  void clear() override;
  void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) override;
  void flip_buffer() override;

 private:
  // --- Initialization helpers ---
  void configure_gpio();
  bool create_lcd_panel();
  void calculate_bcm_timings();
  size_t calculate_bcm_padding(uint8_t bit_plane) const;

  // --- Buffer management ---
  void initialize_blank_buffer(uint8_t *fb);
  void set_brightness_oe_buffer(uint8_t *fb, uint8_t brightness);
  void set_brightness_oe();
  void flush_cache();

  // --- Inline accessors ---
  /// LCD line index for a given row and bit plane
  inline int line_index(int row, int bit) const { return row * bit_depth_ + bit; }

  /// Pointer to byte 0 of a pixel in the framebuffer
  inline uint8_t *pixel_at(uint8_t *fb, int line_idx, int col) const {
    return fb + static_cast<size_t>(line_idx) * line_stride_ + static_cast<size_t>(col) * 2;
  }

  // --- LCD peripheral ---
  esp_lcd_panel_handle_t panel_handle_;

  // --- Framebuffers (owned by LCD driver, freed on panel delete) ---
  uint8_t *lcd_fb_[2];
  int front_idx_;   ///< DMA displays lcd_fb_[front_idx_]
  int active_idx_;  ///< CPU draws to lcd_fb_[active_idx_]
  bool is_double_buffered_;

  // --- BCM parameters ---
  const uint8_t bit_depth_;
  uint8_t lsbMsbTransitionBit_;
  uint32_t actual_clock_hz_;

  // --- Panel geometry (immutable, cached from config) ---
  const uint16_t panel_width_;
  const uint16_t panel_height_;
  const uint16_t layout_rows_;
  const uint16_t layout_cols_;
  const uint16_t virtual_width_;
  const uint16_t virtual_height_;
  const uint16_t dma_width_;

  // --- Coordinate transformation ---
  const Hub75ScanWiring scan_wiring_;
  const Hub75PanelLayout layout_;
  const bool needs_scan_remap_;
  const bool needs_layout_remap_;
  Hub75Rotation rotation_;
  const uint16_t num_rows_;

  // --- LCD timing ---
  uint16_t h_res_;       ///< LCD horizontal resolution (max line width, fixed)
  uint16_t v_res_;       ///< LCD vertical resolution (num_rows × bit_depth)
  size_t line_stride_;   ///< Bytes per LCD line (h_res_ × 3)
  size_t fb_size_;       ///< Total framebuffer size in bytes
  size_t shift_col_;

  // --- BCM padding lookup (indexed by bit plane) ---
  size_t bcm_padding_[HUB75_MAX_BIT_DEPTH];

  // --- Brightness ---
  uint8_t basis_brightness_;
  float intensity_;
  bool transfer_started_;
};

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32P4