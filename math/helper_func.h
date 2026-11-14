#ifndef _HELPER_FUNC_H_
#define _HELPER_FUNC_H_

#include <cmath>

// Helper to normalize angle to [-PI, PI].
// angle: angle in radians.
// Returns the normalized angle in radians.
inline double normalize_angle(double angle)
{
    return angle - ((2.0f * M_PI) * std::floor((angle + M_PI) / (2.0f * M_PI)));
}

// Helper to constrain a value between a min and max.
// val: value to constrain.
// min_val: minimum value.
// max_val: maximum value.
// Returns the constrained value.
inline double constrain(const double val, const double min_val, const double max_val)
{
    return std::min(std::max(val, min_val), max_val);
}

#endif  // _HELPER_FUNC_H_
