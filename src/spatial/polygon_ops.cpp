#include "rge/spatial/polygon_ops.hpp"

#include "rge/geom/polygon_view.hpp"

#include <algorithm>
#include <cmath>

namespace rge::spatial {
namespace {

constexpr double kEpsilon = 1e-9;

bool inside_half_plane(Point2 point, Point2 edge_start, Point2 edge_end,
                       bool keep_left) {
    const double cross = (edge_end - edge_start).cross(point - edge_start);
    if (std::abs(cross) <= kEpsilon) {
        return true;
    }
    return keep_left ? cross >= 0.0 : cross <= 0.0;
}

Point2 intersect_segments(Point2 p1, Point2 p2, Point2 q1, Point2 q2) {
    const Point2 r = p2 - p1;
    const Point2 s = q2 - q1;
    const double denom = r.cross(s);
    if (std::abs(denom) <= kEpsilon) {
        return p2;
    }
    const double t = (q1 - p1).cross(s) / denom;
    return p1 + r * t;
}

Result<std::size_t> clip_with_edge(std::span<const Point2> input,
                                   std::span<Point2> output, Point2 edge_start,
                                   Point2 edge_end, bool keep_left) {
    if (input.empty()) {
        return Result<std::size_t>::ok(0);
    }

    std::size_t out_count = 0;
    Point2 previous = input.back();
    bool previous_inside =
        inside_half_plane(previous, edge_start, edge_end, keep_left);

    for (const Point2& current : input) {
        const bool current_inside =
            inside_half_plane(current, edge_start, edge_end, keep_left);

        if (current_inside) {
            if (!previous_inside) {
                if (out_count >= output.size()) {
                    return Result<std::size_t>::err(GeomError::kBufferTooSmall);
                }
                output[out_count++] =
                    intersect_segments(previous, current, edge_start, edge_end);
            }
            if (out_count >= output.size()) {
                return Result<std::size_t>::err(GeomError::kBufferTooSmall);
            }
            output[out_count++] = current;
        } else if (previous_inside) {
            if (out_count >= output.size()) {
                return Result<std::size_t>::err(GeomError::kBufferTooSmall);
            }
            output[out_count++] =
                intersect_segments(previous, current, edge_start, edge_end);
        }

        previous = current;
        previous_inside = current_inside;
    }

    return Result<std::size_t>::ok(out_count);
}

Result<std::size_t> clip_polygon(std::span<const Point2> subject,
                                 std::span<const Point2> clip_polygon,
                                 ScratchBuffer& scratch) {
    if (subject.size() < 3 || clip_polygon.size() < 3) {
        return Result<std::size_t>::err(GeomError::kDegenerateInput);
    }

    auto buffer = scratch.points();
    if (buffer.size() < subject.size() + clip_polygon.size() + 8) {
        return Result<std::size_t>::err(GeomError::kBufferTooSmall);
    }

    std::span<Point2> input = buffer.subspan(0, subject.size());
    std::span<Point2> output =
        buffer.subspan(subject.size(), buffer.size() - subject.size());

    for (std::size_t i = 0; i < subject.size(); ++i) {
        input[i] = subject[i];
    }
    std::size_t input_count = subject.size();

    for (std::size_t i = 0; i < clip_polygon.size(); ++i) {
        const Point2 edge_start = clip_polygon[i];
        const Point2 edge_end = clip_polygon[(i + 1) % clip_polygon.size()];
        const double area = polygon_signed_area(clip_polygon);
        const bool keep_left = area >= 0.0;

        const Result<std::size_t> clipped = clip_with_edge(
            input.subspan(0, input_count), output, edge_start, edge_end, keep_left);
        if (!clipped.ok()) {
            return clipped;
        }

        input_count = clipped.value();
        if (input_count == 0) {
            return Result<std::size_t>::ok(0);
        }

        for (std::size_t j = 0; j < input_count; ++j) {
            input[j] = output[j];
        }
    }

    for (std::size_t i = 0; i < input_count; ++i) {
        buffer[i] = input[i];
    }
    return Result<std::size_t>::ok(input_count);
}

}  // namespace

double polygon_area(std::span<const Point2> vertices) noexcept {
    return std::abs(polygon_signed_area(vertices));
}

Result<std::size_t> clip_polygon_by_rect(std::span<const Point2> subject,
                                         const Point2& rect_min,
                                         const Point2& rect_max,
                                         ScratchBuffer& scratch) noexcept {
    const Point2 rect[4] = {
        {rect_min.x, rect_min.y},
        {rect_max.x, rect_min.y},
        {rect_max.x, rect_max.y},
        {rect_min.x, rect_max.y},
    };
    return clip_polygon(subject, std::span<const Point2>(rect, 4), scratch);
}

Result<std::size_t> intersect_polygons(std::span<const Point2> subject,
                                       std::span<const Point2> clip,
                                       ScratchBuffer& scratch) noexcept {
    return clip_polygon(subject, clip, scratch);
}

}  // namespace rge::spatial
