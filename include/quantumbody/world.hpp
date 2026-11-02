#pragma once

#include "quantumbody/math.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>
#include <vector>

namespace qb {

struct Sphere { double radius{0.5}; };
struct Box { Vec3 halfExtents{0.5, 0.5, 0.5}; };
using Shape = std::variant<Sphere, Box>;

struct RigidBody {
    std::uint32_t id{};
    Shape shape{Sphere{}};
    Vec3 position{};
    Quat orientation{};
    Vec3 velocity{};
    Vec3 angularVelocity{};
    Vec3 force{};
    Vec3 torque{};
    Vec3 inverseInertiaLocal{};
    double inverseMass{};
    double restitution{0.1};
    double friction{0.6};
    bool isStatic{true};

    static RigidBody sphere(Vec3 position, double radius, double mass, bool isStatic = false);
    static RigidBody box(Vec3 position, Vec3 halfExtents, double mass, bool isStatic = false);
};

struct Contact { std::uint32_t a{}, b{}; Vec3 point{}, normal{}; double penetration{}; };

namespace detail {
struct Aabb { Vec3 min, max; };
struct Proxy { std::uint32_t id; Aabb bounds; };
struct SolverContact {
    Contact contact;
    double normalImpulse{};
    Vec3 tangentImpulse{};
    double bounceSpeed{};
};
} // namespace detail

struct WorldSettings {
    Vec3 gravity{0.0, -9.81, 0.0};
    std::uint32_t solverIterations{10};
    double penetrationSlop{0.005};
    double positionCorrection{0.18};
    double maxLinearSpeed{250.0};
};

struct StepStats {
    std::uint64_t steps{};
    std::uint64_t broadphaseCandidates{};
    std::uint64_t contacts{};
    double stepMilliseconds{};
};

class World {
public:
    explicit World(WorldSettings settings = {});
    std::uint32_t addBody(RigidBody body);
    void addForce(std::uint32_t id, const Vec3& force, const Vec3& worldPoint = {});
    void step(double dt);

    std::vector<RigidBody>& bodies() { return bodies_; }
    const std::vector<RigidBody>& bodies() const { return bodies_; }
    const StepStats& stats() const { return stats_; }
    const WorldSettings& settings() const { return settings_; }
    void resetStats() { stats_ = {}; }

private:
    WorldSettings settings_;
    std::vector<RigidBody> bodies_;
    // Scratch storage is retained across steps and grows to the workload's high-water mark.
    std::vector<detail::Proxy> proxies_;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> broadphasePairs_;
    std::vector<detail::SolverContact> contacts_;
    StepStats stats_;
};

} // namespace qb
