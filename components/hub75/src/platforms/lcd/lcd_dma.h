// SPDX-License-Identifier: MIT
//
// @file lcd_dma.h
// @brief LCD RGB peripheral HUB75 implementation for ESP32-P4 (fixed)

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

  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian) override;
  void clear() override;
  void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
            uint8_t r, uint8_t g, uint8_t b) override;
  void flip_buffer() override;

 private:
  void configure_gpio();
  bool create_lcd_panel();
  void calculate_bcm_timings();
  size_t calculate_bcm_padding(uint8_t bit_plane) const;

  void initialize_blank_buffer(uint8_t *fb);
  void set_brightness_oe_buffer(uint8_t *fb, uint8_t brightness);
  void set_brightness_oe();
  void flush_cache();
  void invert_oe_output();

  /// LCD line index for a given row and bit plane
  inline int line_index(int row, int bit) const {
    return row * bit_depth_ + bit;
  }

  /// Pointer to byte 0 of a 16-bit pixel in the framebuffer
  inline uint8_t *pixel_at(uint8_t *fb, int line_idx, int col) const {
    return fb + static_cast<size_t>(line_idx) * line_stride_
              + static_cast<size_t>(col) * 3;
  }

  esp_lcd_panel_handle_t panel_handle_;

  uint8_t *lcd_fb_[2];
  int front_idx_;
  int active_idx_;
  bool is_double_buffered_;

  const uint8_t bit_depth_;
  uint8_t lsbMsbTransitionBit_;
  uint32_t actual_clock_hz_;

  const uint16_t panel_width_;
  const uint16_t panel_height_;
  const uint16_t layout_rows_;
  const uint16_t layout_cols_;
  const uint16_t virtual_width_;
  const uint16_t virtual_height_;
  const uint16_t dma_width_;

  const Hub75ScanWiring scan_wiring_;
  const Hub75PanelLayout layout_;
  const bool needs_scan_remap_;
  const bool needs_layout_remap_;
  Hub75Rotation rotation_;
  const uint16_t num_rows_;

  uint16_t h_res_;
  uint16_t v_res_;
  size_t line_stride_;
  size_t fb_size_;
  size_t shift_col_;

  /// Number of extra PCLK edges that occur between the last active pixel
  /// and the HSYNC latch. These clock zeros into the shift register.
  /// The shift section is lengthened by this amount so the pixel data
  /// ends up in the correct position after the extra clocks.
  uint16_t blanking_clocks_before_latch_;

  size_t bcm_padding_[HUB75_MAX_BIT_DEPTH];

  uint8_t basis_brightness_;
  float intensity_;
  bool transfer_started_;
};

}  // namespace hub75

#endif