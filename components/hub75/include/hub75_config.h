// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file hub75_config.h
// @brief Compile-time configuration for HUB75 driver

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"

/**
 * IRAM optimization
 * Place hot-path code in instruction RAM to prevent flash cache stalls
 */
#include "esp_attr.h"
#define HUB75_IRAM IRAM_ATTR

/**
 * Compiler optimization attributes
 */
#define HUB75_PURE __attribute__((pure))    // Reads memory, no side effects
#define HUB75_CONST __attribute__((const))  // Pure math, no memory access
#define HUB75_WARN_UNUSED __attribute__((warn_unused_result))

/**
 * Maximum supported bit depth
 * Affects LUT size at compile time
 */
#ifndef HUB75_MAX_BIT_DEPTH
#define HUB75_MAX_BIT_DEPTH 12
#endif

/**
 * Default CIE 1931 LUT shift value
 * Higher values = more precision but darker output
 */
#ifndef HUB75_CIE_SHIFT
#define HUB75_CIE_SHIFT 8
#endif

/**
 * Temporal dithering configuration
 */
#ifndef HUB75_DITHER_SHIFT
#define HUB75_DITHER_SHIFT 8  // Accumulator precision (bits)
#endif

/**
 * Maximum chained panels
 */
#ifndef HUB75_MAX_CHAINED_PANELS
#define HUB75_MAX_CHAINED_PANELS 8
#endif

/**
 * Bit depth configuration (4-12 bits)
 * Set via menuconfig or override: -DHUB75_BIT_DEPTH=10
 */
#ifndef HUB75_BIT_DEPTH
#ifdef CONFIG_HUB75_BIT_DEPTH
#define HUB75_BIT_DEPTH CONFIG_HUB75_BIT_DEPTH
#else
#define HUB75_BIT_DEPTH 8  // Default if no Kconfig
#endif
#endif

/**
 * Gamma mode (0=LINEAR/NONE, 1=CIE1931, 2=GAMMA_2_2)
 * Set via menuconfig or override: -DHUB75_GAMMA_MODE=1
 */
#ifndef HUB75_GAMMA_MODE
#ifdef CONFIG_HUB75_GAMMA_MODE
#define HUB75_GAMMA_MODE CONFIG_HUB75_GAMMA_MODE
#else
#define HUB75_GAMMA_MODE 1  // Default: CIE1931
#endif
#endif

/**
 * Use external framebuffer in PSRAM / SPIRAM
 * Ser via menuconfig or override: -D HUB75_EXTERNAL_FRAMEBUFFERS
 */
#ifndef HUB75_EXTERNAL_FRAMEBUFFERS
#ifdef CONFIG_HUB75_EXTERNAL_FRAMEBUFFERS
#define HUB75_EXTERNAL_FRAMEBUFFERS CONFIG_HUB75_EXTERNAL_FRAMEBUFFERS
#elif defined(CONFIG_SPIRAM) && defined(CONFIG_IDF_TARGET_ESP32P4)
#define HUB75_EXTERNAL_FRAMEBUFFERS 1
#else
#define HUB75_EXTERNAL_FRAMEBUFFERS 0
#endif
#else  // HUB75_EXTERNAL_FRAMEBUFFERS
#if !defined(SOC_SPIRAM_SUPPORTED) && HUB75_EXTERNAL_FRAMEBUFFERS != 0
#pragma message "SOC does not support external framebuffer, disabling..."
#undef HUB75_EXTERNAL_FRAMEBUFFERS
#define HUB75_EXTERNAL_FRAMEBUFFERS 0
#endif
#endif  // HUB75_EXTERNAL_FRAMEBUFFERS



/**
 * Enable parallel output support.
 * Currently up to 3 outputs are supported. This feature is only supported for the lcd implementation for the esp32p4
 */
#ifndef HUB75_ENABLE_PARALLEL_OUTPUT
#ifdef CONFIG_HUB75_ENABLE_PARALLEL_OUTPUT
#define HUB75_ENABLE_PARALLEL_OUTPUT CONFIG_HUB75_ENABLE_PARALLEL_OUTPUT
#else
#define HUB75_ENABLE_PARALLEL_OUTPUT 0
#endif
#endif


/**
 * Use lcd implementation for the esp32p4.
 * This is required for parallel outputs.
 */
#ifndef HUB75_USE_DMA_ENGINE_LCD
#ifdef CONFIG_HUB75_USE_DMA_ENGINE_LCD
#define HUB75_USE_DMA_ENGINE_LCD CONFIG_HUB75_DMA_USE_ENGINE_LCD
#else
#define HUB75_USE_DMA_ENGINE_LCD 0
#endif
#endif

#if HUB75_ENABLE_PARALLEL_OUTPUT != 0 && HUB75_USE_DMA_ENGINE_LCD == 0
#pragma message "Parallel output is enabled but lcd engine is not used, switching..."
#undef HUB75_USE_DMA_ENGINE_LCD
#define HUB75_USE_DMA_ENGINE_LCD 1
#endif

#ifdef __cplusplus
}
#endif
