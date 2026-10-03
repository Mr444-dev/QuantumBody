#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace qb {

constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
    double x{}, y{}, z{};

    constexpr Vec3 operator+ (const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    constexpr Vec3 operator- (const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    constexpr Vec3 operator- () const { return {-x, -y, -z}; }
    constexpr Vec3 operator* (double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/ (double s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 hadamard(const Vec3& v) const { return {x * v.x, y * v.y, z * v.z}; }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    constexpr double lengthSquared() const { return x * x + y * y + z * z; }
    double length() const { return std::sqrt(lengthSquared()); }
    Vec3 normalized(const Vec3& fallback = {1.0, 0.0, 0.0}) const {
        const double l = length();
        return l > kEpsilon ? *this / l : fallback;
    }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

constexpr Vec3 operator*(double s, const Vec3& v) { return v * s; }
constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vec3 abs(const Vec3& v) { return {std::abs(v.x), std::abs(v.y), std::abs(v.z)}; }
inline Vec3 clamp(const Vec3& v, const Vec3& lo, const Vec3& hi) {
    return {std::clamp(v.x, lo.x, hi.x), std::clamp(v.y, lo.y, hi.y), std::clamp(v.z, lo.z, hi.z)};
}

struct Quat {
    double w{1.0}, x{}, y{}, z{};

    static Quat axisAngle(const Vec3& axis, double radians) {
        const Vec3 n = axis.normalized();
        const double h = radians * 0.5;
        const double s = std::sin(h);
        return {std::cos(h), n.x * s, n.y * s, n.z * s};
    }
    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }
    constexpr Quat operator*(const Quat& q) const {
        return {w*q.w - x*q.x - y*q.y - z*q.z,
                w*q.x + x*q.w + y*q.z - z*q.y,
                w*q.y - x*q.z + y*q.w + z*q.x,
                w*q.z + x*q.y - y*q.x + z*q.w};
    }
    Quat normalized() const {
        const double n = std::sqrt(w*w + x*x + y*y + z*z);
        return n > kEpsilon ? Quat{w/n, x/n, y/n, z/n} : Quat{};
    }
    Vec3 rotate(const Vec3& v) const {
        const Vec3 qv{x, y, z};
        const Vec3 t = 2.0 * cross(qv, v);
        return v + w * t + cross(qv, t);
    }
    bool finite() const { return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

inline Vec3 inverseInertiaWorld(const Quat& orientation, const Vec3& inverseLocal, const Vec3& value) {
    const Vec3 local = orientation.conjugate().rotate(value);
    return orientation.rotate(local.hadamard(inverseLocal));
}

} // namespace qb
