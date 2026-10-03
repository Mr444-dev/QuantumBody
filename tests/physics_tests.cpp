#include "quantumbody/scenes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
int failures = 0;
void check(bool condition, std::string_view message) {
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
}
}

int main() {
    {
        qb::World world;
        auto body = qb::RigidBody::sphere({0.0, 10.0, 0.0}, 0.5, 1.0);
        const auto id = world.addBody(body);
        for (int i = 0; i < 120; ++i) world.step(1.0 / 120.0);
        const auto& settled = world.bodies()[id];
        check(std::abs(settled.velocity.y + 9.81) < 0.12, "free fall velocity matches gravity at one second");
        check(settled.position.y < 5.2 && settled.position.y > 4.7, "semi-implicit Euler free fall position is plausible");
    }
    {
        auto a = qb::makeScene("swarm", 123, 18);
        auto b = qb::makeScene("swarm", 123, 18);
        for (int i = 0; i < 180; ++i) { a.step(1.0 / 120.0); b.step(1.0 / 120.0); }
        bool same = a.bodies().size() == b.bodies().size();
        for (std::size_t i = 0; same && i < a.bodies().size(); ++i)
            same = (a.bodies()[i].position - b.bodies()[i].position).length() < 1.0e-12;
        check(same, "fixed seed and step sequence reproduce body positions");
    }
    {
        qb::World world;
        auto floor = qb::RigidBody::box({0.0, -0.5, 0.0}, {4.0, 0.5, 4.0}, 1.0, true);
        world.addBody(floor);
        auto ball = qb::RigidBody::sphere({0.0, 1.0, 0.0}, 0.5, 1.0);
        ball.restitution = 0.0;
        const auto id = world.addBody(ball);
        for (int i = 0; i < 360; ++i) world.step(1.0 / 120.0);
        check(world.bodies()[id].position.y > 0.42, "sphere remains above the static floor after impact");
        check(world.stats().contacts > 0, "contact statistics count generated contacts");
    }
    {
        qb::World world;
        world.addBody(qb::RigidBody::box({0.0, -0.5, 0.0}, {4.0, 0.5, 4.0}, 1.0, true));
        const auto id = world.addBody(qb::RigidBody::box({0.0, 2.0, 0.0}, {0.5, 0.5, 0.5}, 1.0));
        for (int i = 0; i < 360; ++i) world.step(1.0 / 120.0);
        const auto& box = world.bodies()[id];
        check(box.position.y > 0.35 && box.position.y < 0.8, "box SAT contact settles above the floor without a false corner torque");
        check(box.position.finite() && box.orientation.finite(), "box contact keeps the rigid-body state finite");
    }
    {
        auto world = qb::makeScene("stack");
        for (int i = 0; i < 240; ++i) world.step(1.0 / 120.0);
        double lowest = 1.0e9, highest = -1.0e9;
        bool finite = true;
        for (std::size_t i = 1; i < world.bodies().size(); ++i) {
            lowest = std::min(lowest, world.bodies()[i].position.y);
            highest = std::max(highest, world.bodies()[i].position.y);
            finite = finite && world.bodies()[i].position.finite() && world.bodies()[i].orientation.finite();
        }
        check(lowest > 0.25, "box stack stays above the floor after landing");
        check(highest < 5.0, "box stack avoids an explosive launch during settling");
        check(finite, "box stack remains finite through contact resolution");
    }
    {
        qb::World world;
        bool rejected = false;
        try { world.step(0.0); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid time steps are rejected");
    }
    if (failures != 0) return EXIT_FAILURE;
    std::cout << "QuantumBody physics checks passed.\n";
    return EXIT_SUCCESS;
}
