#include "supervisor.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mission {
const char* name(Mode mode) {
    switch (mode) {
      case Mode::Init: return "INIT";
      case Mode::Active: return "ACTIVE";
      case Mode::Hold: return "HOLD";
      case Mode::Land: return "LAND";
      case Mode::Complete: return "COMPLETE";
    }
    return "INVALID";
}
Vec3 enu_to_ned(Vec3 v) { return {v.y, v.x, -v.z}; }
Decision Supervisor::step(const Sample& s) {
    const double values[] = {s.now, s.imu_stamp, s.gnss_stamp, s.vision_stamp,
        s.link_stamp, s.battery, s.position.x, s.position.y, s.position.z,
        s.target.x, s.target.y, s.target.z, s.inference_ms};
    for (double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite_sample");
    if (s.now < 0 || (seen_ && (s.now <= last_time_ || s.sequence <= last_sequence_)))
        throw std::invalid_argument("nonmonotonic_sample");
    for (double stamp : {s.imu_stamp, s.gnss_stamp, s.vision_stamp, s.link_stamp})
        if (stamp < 0 || stamp > s.now + 1e-9) throw std::invalid_argument("invalid_timestamp");
    if (s.battery < 0 || s.battery > 1 || s.inference_ms < 0)
        throw std::invalid_argument("invalid_sensor_range");
    seen_ = true;
    last_time_ = s.now;
    last_sequence_ = s.sequence;
    std::string reason = "healthy";
    const bool geofence = std::hypot(s.position.x, s.position.y) > 20 ||
                          s.position.z < -0.1 || s.position.z > 12;
    const bool bad_target = std::hypot(s.target.x, s.target.y) > 20 ||
                            s.target.z < 0 || s.target.z > 12;
    if (s.battery < 0.20) { mode_ = Mode::Land; reason = "low_battery"; }
    else if (geofence) { mode_ = Mode::Land; reason = "geofence"; }
    else if (bad_target) { mode_ = Mode::Land; reason = "invalid_target"; }
    else if (mode_ == Mode::Land) reason = "land_latched";
    else if (mode_ == Mode::Complete) reason = "mission_complete";
    else {
        if (s.now - s.imu_stamp > 0.15 + 1e-9) reason = "imu_stale";
        else if (s.now - s.gnss_stamp > 0.50 + 1e-9) reason = "gnss_stale";
        else if (s.now - s.vision_stamp > 0.30 + 1e-9) reason = "vision_stale";
        else if (s.now - s.link_stamp > 0.50 + 1e-9) reason = "link_stale";
        else if (s.inference_ms > 50) reason = "inference_overrun";
        if (reason != "healthy") {
            healthy_since_ = -1;
            if (fault_since_ < 0) fault_since_ = s.now;
            mode_ = s.now - fault_since_ >= 2.0 - 1e-9 ? Mode::Land : Mode::Hold;
        } else {
            if (healthy_since_ < 0) healthy_since_ = s.now;
            // A fault episode ends only after 2 s of continuous health. Resetting on
            // the first healthy sample let an intermittent fault flap HOLD/ACTIVE forever.
            if (s.now - healthy_since_ >= 2.0 - 1e-9) fault_since_ = -1;
            const double dwell = activated_ ? 0.5 : 1.0;
            if (mode_ == Mode::Active || s.now - healthy_since_ >= dwell - 1e-9) {
                mode_ = Mode::Active;
                activated_ = true;
                if (s.complete) mode_ = Mode::Complete;
            }
            reason = mode_ == Mode::Init ? "preflight_dwell" :
                     mode_ == Mode::Hold ? "recovery_dwell" :
                     mode_ == Mode::Complete ? "mission_complete" : "healthy";
        }
    }
    Vec3 command{};
    if (mode_ == Mode::Active) {
        command = {s.target.x - s.position.x, s.target.y - s.position.y,
                   s.target.z - s.position.z};
        const double speed = std::sqrt(command.x*command.x + command.y*command.y + command.z*command.z);
        if (speed > 2) { command.x *= 2/speed; command.y *= 2/speed; command.z *= 2/speed; }
    } else if (mode_ == Mode::Land && s.position.z > 0) {
        command.z = -std::min(0.7, s.position.z);
    }
    return {mode_, reason, command};
}
}  // namespace mission
