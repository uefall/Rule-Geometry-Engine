#pragma once

#include <cmath>

namespace rge {

struct Point2 {
    double x{0.0};
    double y{0.0};

    constexpr Point2() = default;
    constexpr Point2(double x_value, double y_value) : x(x_value), y(y_value) {}

    Point2 operator+(const Point2& other) const {
        return {x + other.x, y + other.y};
    }

    Point2 operator-(const Point2& other) const {
        return {x - other.x, y - other.y};
    }

    Point2 operator*(double scalar) const { return {x * scalar, y * scalar}; }

    double dot(const Point2& other) const { return x * other.x + y * other.y; }

    double cross(const Point2& other) const { return x * other.y - y * other.x; }

    double length() const { return std::hypot(x, y); }

    double length_squared() const { return x * x + y * y; }
};

using Vec2 = Point2;

}  // namespace rge
