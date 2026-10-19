#pragma once
namespace AD::Utils
{
float Remap(float val, float inmin, float inmax, float outmin, float outmax);
float Clamp(float val, float min, float max);
float ClampAbs(float val, float range);
bool InRange(float val, float range);
bool InRange(float val, float min, float max);
} // namespace AD::Utils