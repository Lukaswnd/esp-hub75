// SPDX-License-Identifier: MIT
// @file lcd_dma.h
// @brief LCD RGB peripheral HUB75 implementation for ESP32-P4
//
// HSYNC=LAT, OE inverted via GPIO matrix.
// 1 Strand → 16-bit LCD (2 Bytes/Pixel)
// 2-3 Stränge → 24-bit LCD (3 Bytes/Pixel)
//
// Bit-Layout (konsistent für alle Modi):
//   [5:0]   A, B, C, D, E, OE_inv   (Control, immer)
//   [11:6]  S1: R1 G1 B1 R2 G2 B2   (Strang 1, immer)
//   [17:12] S2: R1 G1 B1 R2 G2 B2   (nur bei num_strands≥2, 24-bit)
//   [23:18] S3: R1 G1 B1 R2 G2 B2   (nur bei num_strands≥3, 24-bit)

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
  // --- Init ---
  void configure_gpio();
  bool create_lcd_panel();
  void calculate_bcm_timings();
  size_t calculate_bcm_padding(uint8_t bit_plane) const;

  // --- Buffer ---
  void initialize_blank_buffer(uint8_t *fb);
  void set_brightness_oe_buffer(uint8_t *fb, uint8_t brightness);
  void set_brightness_oe();
  void flush_cache();

  // --- Pixel access ---
  inline int line_index(int row, int bit) const { return row * bit_depth_ + bit; }

  inline uint8_t *pixel_at(uint8_t *fb, int line, int col) const {
    return fb + static_cast<size_t>(line) * line_stride_
              + static_cast<size_t>(col) * bytes_per_pixel_;

  }

  inline uint32_t read_px(const uint8_t *p) const {
    if (bytes_per_pixel_ == 2)
      return *reinterpret_cast<const uint16_t *>(p);
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16);
  }

  inline void write_px(uint8_t *p, uint32_t v) const {
    if (bytes_per_pixel_ == 2) {
      *reinterpret_cast<uint16_t *>(p) = static_cast<uint16_t>(v);
    } else {
      p[0] = uint8_t(v);
      p[1] = uint8_t(v >> 8);
      p[2] = uint8_t(v >> 16);
    }
  }

  // --- LCD ---
  esp_lcd_panel_handle_t panel_handle_;
  uint8_t *lcd_fb_[2];
  int front_idx_;
  int active_idx_;
  bool is_double_buffered_;

  // --- BCM ---
  const uint8_t bit_depth_;
  uint8_t lsbMsbTransitionBit_;
  uint32_t actual_clock_hz_;

  // --- Geometrie (pro Strang, identisch) ---
  const uint16_t panel_width_;
  const uint16_t panel_height_;
  const uint16_t layout_rows_;
  const uint16_t layout_cols_;
  const uint16_t strand_width_;   ///< panel_width × layout_cols
  const uint16_t strand_height_;  ///< panel_height × layout_rows
  const uint16_t dma_width_;
  const uint16_t num_rows_;

  // --- Virtuelles Display (alle Stränge) ---
  const uint16_t virtual_width_;
  const uint16_t virtual_height_;

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
  // --- Multi-Strand ---
  const uint8_t num_strands_;
  const Hub75StrandArrangement strand_arrangement_;
#endif

  // --- LCD-Bus ---
  const uint8_t data_width_;       ///< 16 oder 24
  const uint8_t bytes_per_pixel_;  ///< 2 oder 3

  // --- Koordinaten ---
  const Hub75ScanWiring scan_wiring_;
  const Hub75PanelLayout layout_;
  const bool needs_scan_remap_;
  const bool needs_layout_remap_;
  Hub75Rotation rotation_;

  // --- LCD Timing ---
  uint16_t h_res_;
  uint16_t v_res_;
  size_t line_stride_;
  size_t fb_size_;
  size_t shift_col_;               ///< Feste Spalte wo Shift-Sektion beginnt
  size_t bcm_padding_[HUB75_MAX_BIT_DEPTH];

  // --- Brightness ---
  uint8_t basis_brightness_;
  float intensity_;
  bool transfer_started_;
};

}  // namespace hub75
#endif  // CONFIG_IDF_TARGET_ESP32P4