#include "supervisor.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mission;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
Sample sample(double t) {
    Sample s;
    s.sequence = static_cast<std::uint64_t>(t*1000)+1;
    s.now = s.imu_stamp = s.gnss_stamp = s.vision_stamp = s.link_stamp = t;
    s.position = {0,0,3}; s.target = {8,8,4}; s.inference_ms = 12;
    return s;
}
template<class F> void rejects(F f) {
    bool caught = false;
    try { f(); } catch (const std::invalid_argument&) { caught = true; }
    require(caught, "untrusted input accepted");
}
int main() {
    Supervisor normal;
    require(normal.step(sample(0)).mode == Mode::Init, "preflight must dwell");
    auto a = normal.step(sample(1));
    require(a.mode == Mode::Active, "should become active");
    require(std::sqrt(a.velocity.x*a.velocity.x+a.velocity.y*a.velocity.y+a.velocity.z*a.velocity.z) <= 2.000001, "speed limit");
    auto fault = sample(2); fault.vision_stamp = 1;
    require(normal.step(fault).mode == Mode::Hold, "stale camera must hold");
    require(normal.step(sample(2.1)).mode == Mode::Hold, "recovery must dwell");
    require(normal.step(sample(2.6)).mode == Mode::Active, "healthy recovery");
    auto done = sample(3); done.complete = true;
    require(normal.step(done).mode == Mode::Complete, "completion");
    require(normal.step(sample(4)).mode == Mode::Complete, "completion latch");
    Supervisor interrupted_start;
    auto startup = sample(0); startup.inference_ms = 80;
    require(interrupted_start.step(startup).mode == Mode::Hold, "startup fault");
    interrupted_start.step(sample(0.1));
    require(interrupted_start.step(sample(0.6)).mode == Mode::Hold, "startup recovery still needs full preflight");
    require(interrupted_start.step(sample(1.1)).mode == Mode::Active, "startup recovery full dwell");
    Supervisor loss;
    loss.step(sample(0)); loss.step(sample(1));
    auto b = sample(2); b.link_stamp = 1;
    require(loss.step(b).mode == Mode::Hold, "link hold");
    b = sample(4); b.link_stamp = 1;
    auto land = loss.step(b);
    require(land.mode == Mode::Land && land.velocity.z < 0 && land.velocity.x == 0, "sustained loss must land");
    require(loss.step(sample(5)).mode == Mode::Land, "land must latch");
    // A drifted estimate at or below ground must not stop the descent before touchdown.
    double t = 6;
    for (double z : {0.0, -0.05}) {
        b = sample(t); b.position.z = z; t += 0.5;
        const auto d = loss.step(b);
        require(d.mode == Mode::Land && d.velocity.z <= -0.3 + 1e-9, "land keeps a minimum descent rate");
    }
    b = sample(t); b.position.z = 5;
    require(loss.step(b).velocity.z >= -0.7 - 1e-9, "land descent rate is bounded");
    Supervisor degraded;
    degraded.step(sample(0)); degraded.step(sample(1));
    // Link packets every 0.6 s: each gap goes stale, then briefly recovers.
    Mode last = Mode::Active;
    for (double t = 2, link = 2; t < 6 && last != Mode::Land; t += 0.05) {
        if (t - link >= 0.6 - 1e-9) link = t;
        b = sample(t); b.link_stamp = link; last = degraded.step(b).mode;
    }
    require(last == Mode::Land, "intermittent fault must escalate");
    Supervisor episodes;
    episodes.step(sample(0)); episodes.step(sample(1));
    b = sample(2); b.vision_stamp = 1.5;
    require(episodes.step(b).mode == Mode::Hold, "first dropout holds");
    episodes.step(sample(2.6));
    require(episodes.step(sample(3.1)).mode == Mode::Active, "first dropout recovers");
    require(episodes.step(sample(4.6)).mode == Mode::Active, "sustained health");
    b = sample(4.7); b.vision_stamp = 4.3;
    require(episodes.step(b).mode == Mode::Hold, "2 s of health clears the fault episode");
    Supervisor boundary;
    boundary.step(sample(0)); boundary.step(sample(1));
    b = sample(2); b.vision_stamp = 1.7;
    require(boundary.step(b).mode == Mode::Active, "freshness inclusive boundary");
    b = sample(2.05); b.vision_stamp = 1.7;
    require(boundary.step(b).mode == Mode::Hold, "freshness exceeded");
    for (int kind=0;kind<3;++kind) {
        Supervisor s; b=sample(0);
        if(kind==0) b.battery=0.19;
        if(kind==1) b.position.x=21;
        if(kind==2) b.target.z=13;
        require(s.step(b).mode==Mode::Land, "hard limit must land");
    }
    Supervisor invalid; invalid.step(sample(0));
    rejects([&]{ invalid.step(sample(0)); });
    b=sample(1); b.imu_stamp=2; rejects([&]{invalid.step(b);});
    b=sample(1); b.position.x=std::numeric_limits<double>::quiet_NaN(); rejects([&]{invalid.step(b);});
    b=sample(1); b.battery=2; rejects([&]{invalid.step(b);});
    auto ned=enu_to_ned({1,2,3});
    require(ned.x==2 && ned.y==1 && ned.z==-3, "ENU NED conversion");
    std::cout << "Supervisor contract checks passed\n";
}
