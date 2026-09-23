#include "supervisor.hpp"
#include <iostream>
#include <sstream>
#include <string>
int main() {
    mission::Supervisor supervisor;
    std::string line;
    while (std::getline(std::cin, line)) {
        mission::Sample s;
        long long sequence;
        int complete;
        std::istringstream in(line);
        std::string extra;
        if (!(in >> sequence >> s.now >> s.imu_stamp >> s.gnss_stamp >> s.vision_stamp
              >> s.link_stamp >> s.battery >> s.position.x >> s.position.y >> s.position.z
              >> s.target.x >> s.target.y >> s.target.z >> s.inference_ms >> complete)
              || sequence < 0 || (in >> extra) || (complete != 0 && complete != 1)) {
            std::cerr << "invalid_frame\n";
            return 2;
        }
        s.sequence = static_cast<std::uint64_t>(sequence);
        s.complete = complete != 0;
        try {
            const auto d = supervisor.step(s);
            std::cout << s.sequence << ' ' << mission::name(d.mode) << ' ' << d.reason
                      << ' ' << d.velocity.x << ' ' << d.velocity.y << ' ' << d.velocity.z << std::endl;
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            return 2;  // No command is emitted for untrusted input.
        }
    }
}
