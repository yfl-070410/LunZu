#include "math.h"


int16_t float_to_int16(float f)
{
    /* NaN → 0 */
    if (isnan(f)) return 0;

    /* 饱和 */
    if (f >=  32767.0f) return  32767;
    if (f <= -32768.0f) return -32768;

    /* 四舍五入*/
    return (int16_t)(f >= 0.0f ? f + 0.5f : f - 0.5f);
}
