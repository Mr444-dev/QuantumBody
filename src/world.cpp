#include "quantumbody/world.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace qb {
namespace {

struct Aabb { Vec3 min, max; };
struct Proxy { std::uint32_t id; Aabb bounds; };
struct SolverContact {
    Contact contact;
    double normalImpulse{};
    Vec3 tangentImpulse{};
    double bounceSpeed{};
};

Vec3 boxAxis(const RigidBody& body, int axis) {
    Vec3 local{};
    if (axis == 0) local.x = 1.0;
    else if (axis == 1) local.y = 1.0;
    else local.z = 1.0;
    return body.orientation.rotate(local).normalized();
}

Vec3 halfExtents(const RigidBody& body) { return std::get<Box>(body.shape).halfExtents; }

Vec3 support(const RigidBody& body, const Vec3& direction) {
    if (const auto* sphere = std::get_if<Sphere>(&body.shape))
        return body.position + direction.normalized() * sphere->radius;
    const Vec3 ext = halfExtents(body);
    Vec3 point = body.position;
    for (int i = 0; i < 3; ++i) {
        const Vec3 axis = boxAxis(body, i);
        const double alignment = dot(axis, direction);
        const double sign = alignment > 1.0e-8 ? 1.0 : (alignment < -1.0e-8 ? -1.0 : 0.0);
        const double e = i == 0 ? ext.x : (i == 1 ? ext.y : ext.z);
        point += axis * (sign * e);
    }
    return point;
}

Aabb bounds(const RigidBody& body) {
    if (const auto* sphere = std::get_if<Sphere>(&body.shape)) {
        const Vec3 r{sphere->radius, sphere->radius, sphere->radius};
        return {body.position - r, body.position + r};
    }
    const Vec3 e = halfExtents(body);
    const Vec3 a0 = abs(boxAxis(body, 0));
    const Vec3 a1 = abs(boxAxis(body, 1));
    const Vec3 a2 = abs(boxAxis(body, 2));
    const Vec3 r = a0 * e.x + a1 * e.y + a2 * e.z;
    return {body.position - r, body.position + r};
}

bool overlaps(const Aabb& a, const Aabb& b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x &&
           a.min.y <= b.max.y && a.max.y >= b.min.y &&
           a.min.z <= b.max.z && a.max.z >= b.min.z;
}

bool sphereSphere(const RigidBody& a, const RigidBody& b, Contact& out) {
    const double ra = std::get<Sphere>(a.shape).radius;
    const double rb = std::get<Sphere>(b.shape).radius;
    const Vec3 delta = b.position - a.position;
    const double distance2 = delta.lengthSquared();
    const double sum = ra + rb;
    if (distance2 >= sum * sum) return false;
    const double distance = std::sqrt(std::max(distance2, 0.0));
    out = {a.id, b.id, a.position + delta.normalized() * (ra - 0.5 * (sum - distance)),
           delta.normalized(), sum - distance};
    return true;
}

bool sphereBox(const RigidBody& sphereBody, const RigidBody& boxBody, Contact& out) {
    const double radius = std::get<Sphere>(sphereBody.shape).radius;
    const Vec3 ext = halfExtents(boxBody);
    const Vec3 local = boxBody.orientation.conjugate().rotate(sphereBody.position - boxBody.position);
    const Vec3 closestLocal = clamp(local, -ext, ext);
    const Vec3 closestWorld = boxBody.position + boxBody.orientation.rotate(closestLocal);
    Vec3 towardBox = closestWorld - sphereBody.position;
    double distance = towardBox.length();
    if (distance > radius) return false;
    if (distance <= kEpsilon) {
        const Vec3 gap = ext - abs(local);
        int axis = gap.x < gap.y ? (gap.x < gap.z ? 0 : 2) : (gap.y < gap.z ? 1 : 2);
        const double sign = axis == 0 ? (local.x >= 0.0 ? 1.0 : -1.0) :
                            axis == 1 ? (local.y >= 0.0 ? 1.0 : -1.0) : (local.z >= 0.0 ? 1.0 : -1.0);
        Vec3 outward{};
        if (axis == 0) outward.x = sign;
        if (axis == 1) outward.y = sign;
        if (axis == 2) outward.z = sign;
        towardBox = -boxBody.orientation.rotate(outward);
        distance = 0.0;
    }
    const Vec3 normal = towardBox.normalized(-boxAxis(boxBody, 1));
    out = {sphereBody.id, boxBody.id, closestWorld, normal, radius - distance};
    return true;
}

bool boxBox(const RigidBody& a, const RigidBody& b, Contact& out) {
    const Vec3 ea = halfExtents(a), eb = halfExtents(b);
    const Vec3 delta = b.position - a.position;
    Vec3 bestAxis{};
    double minOverlap = std::numeric_limits<double>::infinity();
    auto testAxis = [&](Vec3 axis) {
        const double n2 = axis.lengthSquared();
        if (n2 < 1.0e-12) return true;
        axis = axis / std::sqrt(n2);
        const double ra = ea.x * std::abs(dot(axis, boxAxis(a, 0))) +
                          ea.y * std::abs(dot(axis, boxAxis(a, 1))) +
                          ea.z * std::abs(dot(axis, boxAxis(a, 2)));
        const double rb = eb.x * std::abs(dot(axis, boxAxis(b, 0))) +
                          eb.y * std::abs(dot(axis, boxAxis(b, 1))) +
                          eb.z * std::abs(dot(axis, boxAxis(b, 2)));
        const double signedDistance = dot(delta, axis);
        const double overlap = ra + rb - std::abs(signedDistance);
        if (overlap <= 0.0) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = signedDistance >= 0.0 ? axis : -axis;
        }
        return true;
    };
    const Vec3 aa[3]{boxAxis(a, 0), boxAxis(a, 1), boxAxis(a, 2)};
    const Vec3 ba[3]{boxAxis(b, 0), boxAxis(b, 1), boxAxis(b, 2)};
    for (const auto& axis : aa) if (!testAxis(axis)) return false;
    for (const auto& axis : ba) if (!testAxis(axis)) return false;
    for (const auto& x : aa) for (const auto& y : ba) if (!testAxis(cross(x, y))) return false;
    const Vec3 pa = support(a, bestAxis);
    const Vec3 pb = support(b, -bestAxis);
    const double volumeA = ea.x * ea.y * ea.z;
    const double volumeB = eb.x * eb.y * eb.z;
    Vec3 tangentAnchor;
    if (a.isStatic != b.isStatic) tangentAnchor = a.isStatic ? b.position : a.position;
    else if (volumeA <= volumeB) tangentAnchor = a.position;
    else tangentAnchor = b.position;
    const double contactPlane = 0.5 * (dot(pa, bestAxis) + dot(pb, bestAxis));
    const Vec3 point = tangentAnchor + bestAxis * (contactPlane - dot(tangentAnchor, bestAxis));
    out = {a.id, b.id, point, bestAxis, minOverlap};
    return true;
}

bool collide(const RigidBody& a, const RigidBody& b, Contact& out) {
    if (std::holds_alternative<Sphere>(a.shape) && std::holds_alternative<Sphere>(b.shape))
        return sphereSphere(a, b, out);
    if (std::holds_alternative<Sphere>(a.shape)) return sphereBox(a, b, out);
    if (std::holds_alternative<Sphere>(b.shape)) {
        const bool hit = sphereBox(b, a, out);
        if (hit) { std::swap(out.a, out.b); out.normal = -out.normal; }
        return hit;
    }
    return boxBox(a, b, out);
}

double effectiveMass(const RigidBody& a, const RigidBody& b, const Vec3& ra, const Vec3& rb, const Vec3& axis) {
    const Vec3 ca = cross(ra, axis), cb = cross(rb, axis);
    return a.inverseMass + b.inverseMass +
           dot(axis, cross(inverseInertiaWorld(a.orientation, a.inverseInertiaLocal, ca), ra) +
                       cross(inverseInertiaWorld(b.orientation, b.inverseInertiaLocal, cb), rb));
}

Vec3 pointVelocity(const RigidBody& body, const Vec3& r) { return body.velocity + cross(body.angularVelocity, r); }

void applyImpulse(RigidBody& a, RigidBody& b, const Vec3& impulse, const Vec3& ra, const Vec3& rb) {
    if (!a.isStatic) {
        a.velocity -= impulse * a.inverseMass;
        a.angularVelocity -= inverseInertiaWorld(a.orientation, a.inverseInertiaLocal, cross(ra, impulse));
    }
    if (!b.isStatic) {
        b.velocity += impulse * b.inverseMass;
        b.angularVelocity += inverseInertiaWorld(b.orientation, b.inverseInertiaLocal, cross(rb, impulse));
    }
}

} // namespace

RigidBody RigidBody::sphere(Vec3 position, double radius, double mass, bool fixed) {
    if (!(radius > 0.0) || !(mass > 0.0) || !std::isfinite(radius) || !std::isfinite(mass))
        throw std::invalid_argument("sphere radius and mass must be positive and finite");
    RigidBody body;
    body.shape = Sphere{radius}; body.position = position; body.isStatic = fixed;
    body.inverseMass = fixed ? 0.0 : 1.0 / mass;
    const double invI = fixed ? 0.0 : 2.5 / (mass * radius * radius);
    body.inverseInertiaLocal = {invI, invI, invI};
    return body;
}

RigidBody RigidBody::box(Vec3 position, Vec3 half, double mass, bool fixed) {
    if (!(half.x > 0.0 && half.y > 0.0 && half.z > 0.0) || !(mass > 0.0) || !half.finite() || !std::isfinite(mass))
        throw std::invalid_argument("box half-extents and mass must be positive and finite");
    RigidBody body;
    body.shape = Box{half}; body.position = position; body.isStatic = fixed;
    body.inverseMass = fixed ? 0.0 : 1.0 / mass;
    body.inverseInertiaLocal = fixed ? Vec3{} : Vec3{
        3.0 / (mass * (half.y*half.y + half.z*half.z)),
        3.0 / (mass * (half.x*half.x + half.z*half.z)),
        3.0 / (mass * (half.x*half.x + half.y*half.y))};
    return body;
}

World::World(WorldSettings settings) : settings_(settings) {
    if (settings_.solverIterations == 0) throw std::invalid_argument("solverIterations must be nonzero");
}

std::uint32_t World::addBody(RigidBody body) {
    body.id = static_cast<std::uint32_t>(bodies_.size());
    bodies_.push_back(body);
    return body.id;
}

void World::addForce(std::uint32_t id, const Vec3& force, const Vec3& worldPoint) {
    if (id >= bodies_.size()) throw std::out_of_range("body id out of range");
    auto& body = bodies_[id];
    if (!body.isStatic) { body.force += force; body.torque += cross(worldPoint - body.position, force); }
}

void World::step(double dt) {
    if (!(dt > 0.0) || !std::isfinite(dt)) throw std::invalid_argument("time step must be positive and finite");
    const auto started = std::chrono::steady_clock::now();
    stats_.broadphaseCandidates = 0;
    stats_.contacts = 0;
    for (auto& body : bodies_) if (!body.isStatic) {
        body.velocity += (settings_.gravity + body.force * body.inverseMass) * dt;
        body.angularVelocity += inverseInertiaWorld(body.orientation, body.inverseInertiaLocal, body.torque) * dt;
        const double speed = body.velocity.length();
        if (speed > settings_.maxLinearSpeed) body.velocity *= settings_.maxLinearSpeed / speed;
        body.position += body.velocity * dt;
        const Quat omega{0.0, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z};
        const Quat derivative = omega * body.orientation;
        body.orientation = Quat{body.orientation.w + 0.5 * dt * derivative.w,
                                body.orientation.x + 0.5 * dt * derivative.x,
                                body.orientation.y + 0.5 * dt * derivative.y,
                                body.orientation.z + 0.5 * dt * derivative.z}.normalized();
        body.force = {}; body.torque = {};
    }

    std::vector<Proxy> proxies;
    proxies.reserve(bodies_.size());
    for (const auto& body : bodies_) proxies.push_back({body.id, bounds(body)});
    std::sort(proxies.begin(), proxies.end(), [](const Proxy& a, const Proxy& b) {
        if (a.bounds.min.x != b.bounds.min.x) return a.bounds.min.x < b.bounds.min.x;
        return a.id < b.id;
    });
    std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs;
    for (std::size_t i = 0; i < proxies.size(); ++i) {
        const auto& a = proxies[i];
        for (std::size_t j = i + 1; j < proxies.size() && proxies[j].bounds.min.x <= a.bounds.max.x; ++j) {
            const auto& b = proxies[j];
            if (bodies_[a.id].isStatic && bodies_[b.id].isStatic) continue;
            if (overlaps(a.bounds, b.bounds)) pairs.emplace_back(std::min(a.id,b.id), std::max(a.id,b.id));
        }
    }
    stats_.broadphaseCandidates += pairs.size();
    std::vector<SolverContact> contacts;
    contacts.reserve(pairs.size());
    for (const auto [aId, bId] : pairs) {
        Contact c;
        if (!collide(bodies_[aId], bodies_[bId], c)) continue;
        const Vec3 ra = c.point - bodies_[aId].position, rb = c.point - bodies_[bId].position;
        const double vn = dot(pointVelocity(bodies_[bId], rb) - pointVelocity(bodies_[aId], ra), c.normal);
        const double e = std::min(bodies_[aId].restitution, bodies_[bId].restitution);
        contacts.push_back({c, 0.0, {}, vn < -1.0 ? -e * vn : 0.0});
    }
    stats_.contacts += contacts.size();
    for (std::uint32_t iteration = 0; iteration < settings_.solverIterations; ++iteration) {
        for (auto& sc : contacts) {
            auto& a = bodies_[sc.contact.a]; auto& b = bodies_[sc.contact.b];
            const Vec3 ra = sc.contact.point - a.position, rb = sc.contact.point - b.position;
            const Vec3 relative = pointVelocity(b, rb) - pointVelocity(a, ra);
            const double kN = effectiveMass(a, b, ra, rb, sc.contact.normal);
            if (kN <= kEpsilon) continue;
            const double bias = settings_.positionCorrection * std::max(sc.contact.penetration - settings_.penetrationSlop, 0.0) / dt;
            const double deltaN = (sc.bounceSpeed + bias - dot(relative, sc.contact.normal)) / kN;
            const double oldN = sc.normalImpulse;
            sc.normalImpulse = std::max(0.0, oldN + deltaN);
            const double appliedN = sc.normalImpulse - oldN;
            applyImpulse(a, b, sc.contact.normal * appliedN, ra, rb);

            const Vec3 after = pointVelocity(b, rb) - pointVelocity(a, ra);
            Vec3 tangent = after - sc.contact.normal * dot(after, sc.contact.normal);
            const double tangentSpeed = tangent.length();
            if (tangentSpeed <= kEpsilon) continue;
            tangent = tangent / tangentSpeed;
            const double kT = effectiveMass(a, b, ra, rb, tangent);
            if (kT <= kEpsilon) continue;
            const double mu = std::sqrt(a.friction * b.friction);
            const Vec3 candidate = sc.tangentImpulse + tangent * (-dot(after, tangent) / kT);
            const double maxFriction = mu * sc.normalImpulse;
            const double candidateLength = candidate.length();
            const Vec3 nextTangent = candidateLength > maxFriction ? candidate * (maxFriction / candidateLength) : candidate;
            applyImpulse(a, b, nextTangent - sc.tangentImpulse, ra, rb);
            sc.tangentImpulse = nextTangent;
        }
    }
    for (const auto& body : bodies_) {
        if (!body.position.finite() || !body.velocity.finite() || !body.orientation.finite())
            throw std::runtime_error("non-finite rigid body state detected");
    }
    ++stats_.steps;
    stats_.stepMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
}

} // namespace qb
