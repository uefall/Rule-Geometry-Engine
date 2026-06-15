#pragma once

#include <array>
#include <cstddef>
#include <span>

#include "rge/geom/point.hpp"

namespace rge {

inline constexpr std::size_t kDefaultScratchCapacity = 256;

class ScratchBuffer {
public:
    ScratchBuffer() = default;

    explicit ScratchBuffer(std::span<Point2> external_buffer)
        : external_(external_buffer), using_external_(true) {}

    std::span<Point2> points() {
        if (using_external_) {
            return external_;
        }
        return std::span<Point2>(internal_.data(), internal_.size());
    }

    std::span<const Point2> points() const {
        if (using_external_) {
            return external_;
        }
        return std::span<const Point2>(internal_.data(), internal_.size());
    }

    std::size_t capacity() const {
        return using_external_ ? external_.size() : internal_.size();
    }

private:
    std::array<Point2, kDefaultScratchCapacity> internal_{};
    std::span<Point2> external_{};
    bool using_external_{false};
};

}  // namespace rge
