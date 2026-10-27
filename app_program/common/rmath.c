/** @file rmath.c
 *
 * @brief Small single-precision maths helpers (see rmath.h).
 */

#include <stdint.h>

#include "rmath.h"

#define RMATH_TWO_PI        (6.28318531f)
#define RMATH_HALF_PI       (1.57079633f)
#define RMATH_SQRT_ITERS    (3U)

float
rmath_absf (float value)
{
    return ((value < 0.0f) ? -value : value);
}

float
rmath_minf (float value_a, float value_b)
{
    return ((value_a < value_b) ? value_a : value_b);
}

float
rmath_maxf (float value_a, float value_b)
{
    return ((value_a > value_b) ? value_a : value_b);
}

float
rmath_clampf (float value, float lower, float upper)
{
    return (rmath_minf(upper, rmath_maxf(lower, value)));
}

/*!
 * @brief Square root: bit-level first guess, then Newton-Raphson.
 *
 * @return sqrt(value), or 0 for value <= 0.
 */
float
rmath_sqrtf (float value)
{
    float result = 0.0f;

    if (value > 0.0f)
    {
        union
        {
            float    as_float;
            uint32_t as_bits;
        } guess;
        uint32_t iter;

        guess.as_float = value;
        guess.as_bits  = 0x1FBD1DF5UL + (guess.as_bits >> 1);
        result = guess.as_float;

        for (iter = 0U; iter < RMATH_SQRT_ITERS; iter++)
        {
            result = 0.5f * (result + (value / result));
        }
    }

    return (result);
}

/*!
 * @brief Sine: reduce to [-pi/2, pi/2], then a 9th-order series.
 */
float
rmath_sinf (float angle_rad)
{
    float x_val = angle_rad;
    float x_sq;

    while (x_val > RMATH_PI)
    {
        x_val -= RMATH_TWO_PI;
    }

    while (x_val < -RMATH_PI)
    {
        x_val += RMATH_TWO_PI;
    }

    if (x_val > RMATH_HALF_PI)
    {
        x_val = RMATH_PI - x_val;
    }
    else if (x_val < -RMATH_HALF_PI)
    {
        x_val = -RMATH_PI - x_val;
    }
    else
    {
        /* Already in range. */
    }

    x_sq = x_val * x_val;

    return (x_val * (1.0f - (x_sq / 6.0f) *
                     (1.0f - (x_sq / 20.0f) *
                      (1.0f - (x_sq / 42.0f) *
                       (1.0f - (x_sq / 72.0f))))));
}

float
rmath_cosf (float angle_rad)
{
    return (rmath_sinf(angle_rad + RMATH_HALF_PI));
}

/*!
 * @brief Tangent as sin/cos.  Caller keeps away from +-pi/2.
 */
float
rmath_tanf (float angle_rad)
{
    return (rmath_sinf(angle_rad) / rmath_cosf(angle_rad));
}

/*!
 * @brief atan(z) for |z| <= 1, polynomial (max error ~1e-5 rad).
 */
static float
rmath_atan_unit (float z_val)
{
    float const z_sq = z_val * z_val;

    return (z_val * (0.9998660f + z_sq * (-0.3302995f + z_sq *
            (0.1801410f + z_sq * (-0.0851330f + z_sq * 0.0208351f)))));
}

/*!
 * @brief Four-quadrant arctangent of y/x, radians in [-pi, pi].
 */
float
rmath_atan2f (float y_value, float x_value)
{
    float const abs_y  = rmath_absf(y_value);
    float const abs_x  = rmath_absf(x_value);
    float       result = 0.0f;

    if ((abs_x > 0.0f) || (abs_y > 0.0f))
    {
        if (abs_x >= abs_y)
        {
            result = rmath_atan_unit(abs_y / abs_x);
        }
        else
        {
            result = RMATH_HALF_PI - rmath_atan_unit(abs_x / abs_y);
        }

        if (x_value < 0.0f)
        {
            result = RMATH_PI - result;
        }

        if (y_value < 0.0f)
        {
            result = -result;
        }
    }

    return (result);
}

/*** end of file ***/
