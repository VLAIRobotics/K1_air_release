#pragma once
#include <array>
#include <cstddef>

// Historical fixture data only: not installed and never used by the driver.
namespace legacy_test {
struct JointPositionLimits { double lower; double upper; };
// Values from x_air_perfect, including the mirrored left-arm J1 range.
inline constexpr std::array<JointPositionLimits, 7> kArmPositionLimits{{
    {-1.7, 3.4},
    {-1.74532925199, 1.74532925199},
    {-1.57079632679, 1.57079632679},
    {0.0, 2.4},
    {-1.57079632679, 1.57079632679},
    {-0.7, 0.7},
    {-1.57079632679, 1.57079632679},
}};
inline constexpr JointPositionLimits joint_position_limits(std::size_t joint, bool left_arm) {
  return joint == 0 && left_arm ? JointPositionLimits{-3.490658504, 1.745329252}
                                : kArmPositionLimits.at(joint);
}
}  // namespace legacy_test
