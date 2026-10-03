#include "quantumbody/scenes.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

namespace qb {

World makeScene(std::string_view name, std::uint32_t seed, std::uint32_t objectCount, WorldSettings settings) {
    World world(settings);
    auto floor = RigidBody::box({0.0, -0.5, 0.0}, {24.0, 0.5, 24.0}, 1.0, true);
    floor.friction = 0.85;
    world.addBody(floor);

    if (name == "stack") {
        constexpr int side = 5;
        for (int y = 0; y < side; ++y) for (int z = 0; z < side; ++z) for (int x = 0; x < side; ++x) {
            auto body = RigidBody::box({(x - 2) * 1.04, 0.52 + y * 1.04, (z - 2) * 1.04}, {0.5, 0.5, 0.5}, 1.0);
            body.restitution = 0.08;
            body.friction = 0.72;
            world.addBody(body);
        }
    } else if (name == "swarm") {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> radius(0.16, 0.31);
        std::uniform_real_distribution<double> jitter(-0.035, 0.035);
        const std::uint32_t count = objectCount > 0 ? objectCount : 240;
        const int width = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(count))));
        for (std::uint32_t i = 0; i < count; ++i) {
            const int x = static_cast<int>(i % static_cast<std::uint32_t>(width));
            const int z = static_cast<int>(i / static_cast<std::uint32_t>(width));
            const double r = radius(rng);
            auto body = RigidBody::sphere({(x - width/2) * 0.72 + jitter(rng), 1.0 + (i / static_cast<std::uint32_t>(width)) * 0.76, (z - width/2) * 0.72 + jitter(rng)}, r, r*r*r*30.0);
            body.restitution = 0.25;
            body.friction = 0.48;
            world.addBody(body);
        }
    } else if (name == "drop") {
        for (int i = 0; i < 12; ++i) {
            auto body = RigidBody::sphere({0.0, 8.0 + i * 1.15, 0.0}, 0.5, 1.0);
            body.restitution = 0.72;
            world.addBody(body);
        }
    } else if (name == "failure-lab") {
        auto fast = RigidBody::sphere({-8.0, 3.0, 0.0}, 0.2, 0.25);
        fast.velocity = {85.0, 0.0, 0.0};
        world.addBody(fast);
        auto wall = RigidBody::box({0.0, 2.0, 0.0}, {0.08, 2.0, 2.0}, 1.0, true);
        world.addBody(wall);
        auto far = RigidBody::sphere({1.0e6, 20.0, 0.0}, 0.4, 1.0);
        world.addBody(far);
    } else {
        throw std::invalid_argument("unknown scene; choose stack, swarm, drop, or failure-lab");
    }
    return world;
}

} // namespace qb
