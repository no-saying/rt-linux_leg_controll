#ifndef P112_ESTIMATION_H
#define P112_ESTIMATION_H
#include <array>
#include <cstddef>
namespace p112 {
using Vec2 = std::array<double,2>;
using Vec3 = std::array<double,3>;
using Vec6 = std::array<double,6>;
// Attitude is roll,pitch,yaw and bias-corrected angular velocity, in radians.
class MotionImu {
public:
    explicit MotionImu(std::size_t alignment_samples = 1000);
    void reset() noexcept;
    bool step(const Vec6& sample, double dt, bool& ready, Vec6& attitude) noexcept;
private:
    std::size_t alignment_samples_, count_{};
    Vec6 sum_{}, attitude_{};
    Vec3 bias_{}, accel_{};
    std::array<double,4> q_{}; // w,x,y,z
    bool ready_{};
};
class TurnOdometryHold {
public:
    TurnOdometryHold(double dt=.01, double gain=0., double max_speed=.05,
                     double max_acceleration=.05, double deadband=.005);
    void reset() noexcept;
    bool step(const Vec2& velocity, double heading, bool enabled, double& correction) noexcept;
private:
    double dt_,gain_,max_speed_,max_acceleration_,deadband_,correction_{};
    Vec2 position_{},previous_{};
    bool has_previous_{};
};
}
#endif
