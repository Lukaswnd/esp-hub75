// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file hub75_driver.cpp
// @brief Main driver implementation

#include "hub75.h"
#include "../color/color_lut.h"
#include "../color/color_convert.h"
#include "../drivers/driver_init.h"
#include "../panels/rotation.h"
#include "../panels/scan_patterns.h"
#include "../platforms/platform_dma.h"
#include "../platforms/platform_detect.h"

// Include platform-specific DMA implementation
#ifdef HUB75_DMA_ENGINE_GDMA
#include "../platforms/gdma/gdma_dma.h"
#elif defined(HUB75_DMA_ENGINE_I2S)
#include "../platforms/i2s/i2s_dma.h"
#elif defined(HUB75_DMA_ENGINE_PARLIO)
#include "../platforms/parlio/parlio_dma.h"
#elif defined(HUB75_DMA_ENGINE_LCD)
#include "../platforms/lcd/lcd_dma.h"
#endif

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <utility>

static const char *const TAG = "HUB75";

using namespace hub75;

// Select platform implementation
#ifdef HUB75_DMA_ENGINE_GDMA
using PlatformDMAImpl = GdmaDma;
#elif defined(HUB75_DMA_ENGINE_I2S)
using PlatformDMAImpl = I2sDma;
#elif defined(HUB75_DMA_ENGINE_PARLIO)
using PlatformDMAImpl = ParlioDma;
#elif defined(HUB75_DMA_ENGINE_LCD)
using PlatformDMAImpl = LcdDma;
#endif

// ============================================================================
// Constructor / Destructor
// ============================================================================
#if HUB75_ENABLE_PARALLEL_OUTPUT == 1
Hub75Driver::Hub75Driver(const Hub75Config &config) : 
      config_(config), running_(false), dma_(nullptr),
      virtual_width_(compute_virtual_width(config)),
      virtual_height_(compute_virtual_height(config)) 
      {
  ESP_LOGI(TAG, "Driver created for %s (%s)", getPlatformName(), getDMAEngineName());
  ESP_LOGI("HUB75", "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d%s",
           config_.panel_width, config_.panel_height,
           config_.layout_cols, config_.layout_rows,
           virtual_width_, virtual_height_,
           config_.num_strands > 1
               ? (config_.strand_arrangement == Hub75StrandArrangement::ROWS
                  ? " (multi-strand ROWS)" : " (multi-strand COLUMNS)")
               : "");
  ESP_LOGI(TAG, "Config: %u-bit depth (compile-time), %u row addresses, four-scan: %s", HUB75_BIT_DEPTH,
           (unsigned int) get_effective_num_rows(config_.scan_wiring, config_.panel_height),
           is_four_scan_wiring(config_.scan_wiring) ? "yes" : "no");
}
#else
Hub75Driver::Hub75Driver(const Hub75Config &config) : config_(config), running_(false), dma_(nullptr)      {
  ESP_LOGI(TAG, "Driver created for %s (%s)", getPlatformName(), getDMAEngineName());
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d", (unsigned int) config_.panel_width,
           (unsigned int) config_.panel_height, (unsigned int) config_.layout_cols, (unsigned int) config_.layout_rows,
           (unsigned int) (config_.panel_width * config_.layout_cols),
           (unsigned int) (config_.panel_height * config_.layout_rows));
  ESP_LOGI(TAG, "Config: %u-bit depth (compile-time), %u row addresses, four-scan: %s", HUB75_BIT_DEPTH,
           (unsigned int) get_effective_num_rows(config_.scan_wiring, config_.panel_height),
           is_four_scan_wiring(config_.scan_wiring) ? "yes" : "no");
}
#endif

Hub75Driver::~Hub75Driver() { end(); }

// ============================================================================
// Initialization
// ============================================================================

bool Hub75Driver::begin() {
  if (running_) {
    ESP_LOGW(TAG, "Already running");
    return true;
  }

  ESP_LOGI(TAG, "Initializing Hub75 driver...");

  // Validate configuration
  if (config_.panel_width == 0 || config_.panel_height == 0) {
    ESP_LOGE(TAG, "Invalid panel dimensions");
    return false;
  }
  if (config_.layout_rows == 0 || config_.layout_cols == 0) {
    ESP_LOGE(TAG, "Invalid panel layout");
    return false;
  }

  // Initialize shift driver chips (panel-level, platform-agnostic)
  // Must happen before DMA starts transmitting data
  esp_err_t err = DriverInit::initialize(config_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Shift driver initialization failed: %s", esp_err_to_name(err));
    return false;
  }

  // Create platform-specific DMA implementation
  dma_ = new PlatformDMAImpl(config_);
  if (!dma_ || !dma_->init()) {
    ESP_LOGE(TAG, "Failed to initialize DMA engine");
    return false;
  }

  // Start DMA transfer
  dma_->start_transfer();

  running_ = true;
  ESP_LOGI(TAG, "Driver started successfully");
  return true;
}

void Hub75Driver::end() {
  if (!running_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping driver...");

  // Shutdown DMA
  if (dma_) {
    dma_->shutdown();
    delete dma_;
    dma_ = nullptr;
  }

  running_ = false;
  ESP_LOGI(TAG, "Hub75 driver stopped");
}

// ============================================================================
// Pixel Drawing
// ============================================================================

HUB75_IRAM void Hub75Driver::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                         Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
  // Forward to platform DMA layer (handles LUT and buffer writes)
  if (dma_) {
    dma_->draw_pixels(x, y, w, h, buffer, format, color_order, big_endian);
  }
}

HUB75_IRAM void Hub75Driver::set_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
  // Single pixel is just a 1x1 draw_pixels call with RGB888 format
  uint8_t rgb[3] = {r, g, b};
  draw_pixels(x, y, 1, 1, rgb, Hub75PixelFormat::RGB888, Hub75ColorOrder::RGB, false);
}

void Hub75Driver::clear() {
  // Forward to platform DMA layer
  if (dma_) {
    dma_->clear();
  }
}

HUB75_IRAM void Hub75Driver::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
  // Forward to platform DMA layer
  if (dma_) {
    dma_->fill(x, y, w, h, r, g, b);
  }
}

// ============================================================================
// Double Buffering
// ============================================================================

void Hub75Driver::flip_buffer() {
  if (!config_.double_buffer) {
    ESP_LOGW(TAG, "flip_buffer() called but double buffering not enabled");
    return;
  }

  if (!dma_) {
    ESP_LOGE(TAG, "flip_buffer() called but DMA not initialized");
    return;
  }

  dma_->flip_buffer();
}

// ============================================================================
// Display Rotation
// ============================================================================

void Hub75Driver::set_rotation(Hub75Rotation rotation) {
  config_.rotation = rotation;
  if (dma_) {
    dma_->set_rotation(rotation);
  }
}

Hub75Rotation Hub75Driver::get_rotation() const { return config_.rotation; }

// ============================================================================
// Color Configuration
// ============================================================================

void Hub75Driver::set_brightness(uint8_t brightness) {
  config_.brightness = brightness;

  // Update basis brightness in DMA layer (platform-specific implementation)
  if (dma_) {
    dma_->set_basis_brightness(brightness);
  }
}

uint8_t Hub75Driver::get_brightness() const { return config_.brightness; }

void Hub75Driver::set_intensity(float intensity) {
  if (dma_) {
    dma_->set_intensity(intensity);
  }
}

// ============================================================================
// Information
// ============================================================================

#if HUB75_ENABLE_PARALLEL_OUTPUT == 1

uint16_t Hub75Driver::get_width() const {
  // Return rotation-aware dimensions using cached strand-aware virtual size
  return hub75::RotationTransform::get_rotated_width(
      virtual_width_, virtual_height_, config_.rotation);
}

uint16_t Hub75Driver::get_height() const {
  // Return virtual height with rotation applied
  return hub75::RotationTransform::get_rotated_height(
        virtual_width_, virtual_height_, config_.rotation);
}

uint16_t Hub75Driver::compute_virtual_width(const Hub75Config &cfg) {
  const uint16_t strand_w = cfg.panel_width * cfg.layout_cols;
  const uint8_t ns = (cfg.num_strands >= 1 && cfg.num_strands <= 3) ? cfg.num_strands : 1;
  if (ns > 1 && cfg.strand_arrangement == Hub75StrandArrangement::COLUMNS)
    return strand_w * ns;
  return strand_w;
}

uint16_t Hub75Driver::compute_virtual_height(const Hub75Config &cfg) {
  const uint16_t strand_h = cfg.panel_height * cfg.layout_rows;
  const uint8_t ns = (cfg.num_strands >= 1 && cfg.num_strands <= 3) ? cfg.num_strands : 1;
  if (ns > 1 && cfg.strand_arrangement == Hub75StrandArrangement::ROWS)
    return strand_h * ns;
  return strand_h;
}

#else

uint16_t Hub75Driver::get_width() const {
  // Return virtual width with rotation applied
  uint16_t phys_w = config_.panel_width * config_.layout_cols;
  uint16_t phys_h = config_.panel_height * config_.layout_rows;
  return RotationTransform::get_rotated_width(phys_w, phys_h, config_.rotation);
}

uint16_t Hub75Driver::get_height() const {
  // Return virtual height with rotation applied
  uint16_t phys_w = config_.panel_width * config_.layout_cols;
  uint16_t phys_h = config_.panel_height * config_.layout_rows;
  return RotationTransform::get_rotated_height(phys_w, phys_h, config_.rotation);
}

#endif


bool Hub75Driver::is_running() const { return running_; }