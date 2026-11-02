#include "quantumbody/scenes.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string_view>

namespace {
std::atomic<bool> trackAllocations{false};
std::atomic<std::size_t> allocationCount{0};
}

void* operator new(std::size_t size) {
    if (trackAllocations.load(std::memory_order_relaxed))
        allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* memory = std::malloc(size == 0 ? 1 : size)) return memory;
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
    if (trackAllocations.load(std::memory_order_relaxed))
        allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* memory = std::malloc(size == 0 ? 1 : size)) return memory;
    throw std::bad_alloc();
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

namespace {
int failures = 0;

void check(bool condition, std::string_view message) {
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
}

void core() {
    qb::World world;
    auto body = qb::RigidBody::sphere({0.0, 10.0, 0.0}, 0.5, 1.0);
    const auto id = world.addBody(body);
    for (int i = 0; i < 120; ++i) world.step(1.0 / 120.0);
    const auto& settled = world.bodies()[id];
    check(std::abs(settled.velocity.y + 9.81) < 0.12, "free fall velocity matches gravity at one second");
    check(settled.position.y < 5.2 && settled.position.y > 4.7, "semi-implicit Euler free fall position is plausible");

    bool rejected = false;
    try { world.step(0.0); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "invalid time steps are rejected");
}

void determinism() {
    auto a = qb::makeScene("swarm", 123, 18);
    auto b = qb::makeScene("swarm", 123, 18);
    for (int i = 0; i < 180; ++i) { a.step(1.0 / 120.0); b.step(1.0 / 120.0); }
    bool same = a.bodies().size() == b.bodies().size();
    for (std::size_t i = 0; same && i < a.bodies().size(); ++i)
        same = (a.bodies()[i].position - b.bodies()[i].position).length() < 1.0e-12;
    check(same, "fixed seed and step sequence reproduce body positions");
}

void sphereFloor() {
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

void boxFloor() {
    qb::World world;
    world.addBody(qb::RigidBody::box({0.0, -0.5, 0.0}, {4.0, 0.5, 4.0}, 1.0, true));
    const auto id = world.addBody(qb::RigidBody::box({0.0, 2.0, 0.0}, {0.5, 0.5, 0.5}, 1.0));
    for (int i = 0; i < 360; ++i) world.step(1.0 / 120.0);
    const auto& box = world.bodies()[id];
    check(box.position.y > 0.35 && box.position.y < 0.8, "box SAT contact settles above the floor without a false corner torque");
    check(box.position.finite() && box.orientation.finite(), "box contact keeps the rigid-body state finite");
}

void boxStack() {
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

void TestSphereCenterDeepInsideOBB() {
    qb::WorldSettings settings;
    settings.gravity = {};
    settings.solverIterations = 1;
    settings.penetrationSlop = 0.0;
    settings.positionCorrection = 1.0;
    qb::World world(settings);
    const auto sphere = world.addBody(qb::RigidBody::sphere({0.0, 0.0, 0.0}, 0.25, 1.0));
    auto box = qb::RigidBody::box({0.0, 0.0, 0.0}, {2.0, 0.25, 1.0}, 1.0, true);
    box.orientation = qb::Quat::axisAngle({0.0, 0.0, 1.0}, qb::kPi * 0.5);
    world.addBody(box);

    world.step(0.1);
    check(world.stats().contacts == 1, "centered sphere produces one sphere-OBB contact");
    check(world.bodies()[sphere].velocity.x < -4.9,
          "deep interior fallback uses the nearest local +Y face and radius-plus-face-distance penetration");
    check(std::abs(world.bodies()[sphere].velocity.y) < 1.0e-12 &&
          std::abs(world.bodies()[sphere].velocity.z) < 1.0e-12,
          "deep interior fallback rotates the ejection axis into world space");
}

void TestNoStepAllocationsAfterWarmup() {
    qb::WorldSettings settings;
    settings.gravity = {};
    settings.positionCorrection = 0.0;
    qb::World world(settings);
    world.addBody(qb::RigidBody::sphere({0.0, 0.0, 0.0}, 0.2, 1.0));
    for (int i = 0; i < 16; ++i)
        world.addBody(qb::RigidBody::box({0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 1.0, true));

    for (int i = 0; i < 8; ++i) world.step(1.0 / 120.0);
    allocationCount.store(0, std::memory_order_relaxed);
    trackAllocations.store(true, std::memory_order_relaxed);
    for (int i = 0; i < 128; ++i) world.step(1.0 / 120.0);
    trackAllocations.store(false, std::memory_order_relaxed);
    check(allocationCount.load(std::memory_order_relaxed) == 0,
          "steady-state World::step performs zero global heap allocations after scratch buffers warm up");
}

struct Suite { std::string_view name; void (*run)(); };
constexpr std::array suites{
    Suite{"core", core},
    Suite{"determinism", determinism},
    Suite{"sphere_floor", sphereFloor},
    Suite{"box_floor", boxFloor},
    Suite{"box_stack", boxStack},
    Suite{"deep_inside_obb", TestSphereCenterDeepInsideOBB},
    Suite{"step_allocations", TestNoStepAllocationsAfterWarmup},
};
}

int main(int argc, char** argv) {
    std::string_view selected;
    if (argc == 3 && std::string_view(argv[1]) == "--suite") selected = argv[2];
    else if (argc != 1) {
        std::cerr << "Usage: quantumbody_tests [--suite NAME]\n";
        return EXIT_FAILURE;
    }

    bool found = selected.empty();
    for (const auto& suite : suites) {
        if (!selected.empty() && suite.name != selected) continue;
        found = true;
        failures = 0;
        suite.run();
        if (failures != 0) return EXIT_FAILURE;
        std::cout << "PASS: " << suite.name << '\n';
    }
    if (!found) {
        std::cerr << "Unknown suite: " << selected << "\nAvailable suites:";
        for (const auto& suite : suites) std::cerr << ' ' << suite.name;
        std::cerr << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
