#pragma once

#include <algorithm>
#include <cmath>

namespace docksim::render {

// float32 3-vector. The kernels work in float32 local metres.
// The constructor keeps Vec3{x, y, z} positional, as in the OpenCL kernels it mirrors.
struct Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;

    constexpr Vec3() noexcept = default;
    constexpr Vec3(float x_, float y_, float z_) noexcept : x(x_), y(y_), z(z_) {}

    constexpr Vec3& operator+=(const Vec3& o) noexcept {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    constexpr Vec3& operator*=(float s) noexcept {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    constexpr Vec3& operator*=(const Vec3& o) noexcept {
        x *= o.x;
        y *= o.y;
        z *= o.z;
        return *this;
    }
    friend constexpr Vec3 operator+(Vec3 a, const Vec3& b) noexcept { return a += b; }
    friend constexpr Vec3 operator-(const Vec3& a, const Vec3& b) noexcept {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    friend constexpr Vec3 operator*(Vec3 a, float s) noexcept { return a *= s; }
    friend constexpr Vec3 operator*(float s, Vec3 a) noexcept { return a *= s; }
    friend constexpr Vec3 operator*(Vec3 a, const Vec3& b) noexcept { return a *= b; }
    friend constexpr Vec3 operator/(const Vec3& a, const Vec3& b) noexcept {
        return {a.x / b.x, a.y / b.y, a.z / b.z};
    }
    friend constexpr Vec3 operator/(const Vec3& a, float s) noexcept {
        return {a.x / s, a.y / s, a.z / s};
    }
};

[[nodiscard]] constexpr float dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vec3 mix(const Vec3& a, const Vec3& b, float t) noexcept {
    return a * (1.0F - t) + b * t;
}

[[nodiscard]] constexpr float smoothstep(float edge0, float edge1, float x) noexcept {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

}  // namespace docksim::render
