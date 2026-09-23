#pragma once
#include <cstdint>
#include <string>

namespace mission {
struct Vec3 { double x{}, y{}, z{}; };
// All timestamps are seconds in one monotonically increasing simulation clock.
struct Sample {
    std::uint64_t sequence{};
    double now{}, imu_stamp{}, gnss_stamp{}, vision_stamp{}, link_stamp{};
    double battery{1.0};
    Vec3 position{}, target{};  // ENU, metres
    double inference_ms{};
    bool complete{};
};
enum class Mode { Init, Active, Hold, Land, Complete };
struct Decision { Mode mode; std::string reason; Vec3 velocity; };
const char* name(Mode mode);
Vec3 enu_to_ned(Vec3 value);
class Supervisor {
 public:
    Decision step(const Sample& sample);
 private:
    Mode mode_{Mode::Init};
    double healthy_since_{-1}, fault_since_{-1}, last_time_{-1};
    std::uint64_t last_sequence_{};
    bool seen_{}, activated_{};
};
}  // namespace mission
