/** @file rmath.h
 *
 * @brief Small single-precision maths helpers.
 *
 * The template links newlib's libc but not libm, and builds with
 * -ffreestanding (no compiler built-ins), so sqrtf(), atan2f(), sinf(),
 * fabsf(), fminf() ... are not available.  These replace them with
 * accuracy well beyond what the sensors need (< 1e-4 relative).
 */

#ifndef RMATH_H
#define RMATH_H

#define RMATH_PI            (3.14159265f)
#define RMATH_DEG_TO_RAD    (0.017453292f)
#define RMATH_RAD_TO_DEG    (57.29577951f)

float rmath_absf(float value);
float rmath_minf(float value_a, float value_b);
float rmath_maxf(float value_a, float value_b);
float rmath_clampf(float value, float lower, float upper);
float rmath_sqrtf(float value);
float rmath_sinf(float angle_rad);
float rmath_cosf(float angle_rad);
float rmath_tanf(float angle_rad);
float rmath_atan2f(float y_value, float x_value);

#endif /* RMATH_H */

/*** end of file ***/
