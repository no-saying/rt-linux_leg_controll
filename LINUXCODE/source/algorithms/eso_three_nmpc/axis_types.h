#ifndef P112_AXIS_TYPES_H
#define P112_AXIS_TYPES_H

#include <array>

namespace p112 {

enum class DecoupledAxis : unsigned char { roll = 0, pitch = 1, yaw = 2 };
struct AxisState4 { std::array<double, 4> value{}; };
struct AxisTorque2 { std::array<double, 2> value{}; };

} // namespace p112

#endif
