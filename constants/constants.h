#pragma once

#include <cmath>

/// @file constants.h
/// @brief Shared physical constants and unit conversions used across the firmware.
///
/// Angular unit conversions
/// ------------------------
/// RPM_TO_RAD_S   — multiply rpm  by this to get rad/s
/// RAD_S_TO_RPM   — multiply rad/s by this to get rpm
/// RAD_S2_TO_RPS2 — multiply rad/s² by this to get revolutions per second²
/// RAD_TO_DEG     — multiply rad  by this to get degrees

// ── Angular unit conversions ──────────────────────────────────────────
constexpr float RPM_TO_RAD_S   = 2.0f * M_PI / 60.0f;   // rpm → rad/s
constexpr float RAD_S_TO_RPM   = 60.0f / (2.0f * M_PI); // rad/s → rpm
constexpr float RAD_S2_TO_RPS2 = 1.0f / (2.0f * M_PI);  // rad/s² → rev/s²
constexpr float RAD_TO_DEG     = 180.0f / M_PI;         // rad → deg
