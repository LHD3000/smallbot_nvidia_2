#include "asio_driver/utils/range.hpp"
#include <cassert>
#include <valarray>
namespace AD::Utils
{
float Remap(float val, float inmin, float inmax, float outmin, float outmax)
{
    return (val - inmin) * (outmax - outmin) / (inmax - inmin) + outmin;
}
float Clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
float ClampAbs(float val, float range)
{
    assert(range > 0);
    return Clamp(val, -range, range);
}

bool InRange(float val, float range)
{
    assert(range > 0);
    return std::abs(val) <= range;
}

bool InRange(float val, float min, float max)
{
    return val >= min && val <= max;
}
} // namespace AD::Utils
