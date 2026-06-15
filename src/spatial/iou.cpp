#include "rge/spatial/iou.hpp"

#include "rge/spatial/polygon_ops.hpp"

namespace rge::spatial {

Result<double> intersection_area(BBox a, BBox b) noexcept {
    if (!a.is_valid() || !b.is_valid()) {
        return Result<double>::err(GeomError::kDegenerateInput);
    }
    const BBox intersection = a.intersect_with(b);
    return Result<double>::ok(intersection.area());
}

Result<double> iou(BBox a, BBox b) noexcept {
    const Result<double> intersection = intersection_area(a, b);
    if (!intersection.ok()) {
        return intersection;
    }
    const double union_area = a.area() + b.area() - intersection.value();
    if (union_area <= 0.0) {
        return Result<double>::ok(0.0);
    }
    return Result<double>::ok(intersection.value() / union_area);
}

Result<double> polygon_intersection_area(PolygonView subject, PolygonView clip,
                                         ScratchBuffer& scratch) noexcept {
    if (subject.size() < 3 || clip.size() < 3) {
        return Result<double>::err(GeomError::kDegenerateInput);
    }

    const Result<std::size_t> clipped =
        intersect_polygons(subject.vertices, clip.vertices, scratch);
    if (!clipped.ok()) {
        return Result<double>::err(clipped.error());
    }
    if (clipped.value() < 3) {
        return Result<double>::ok(0.0);
    }
    return Result<double>::ok(
        polygon_area(scratch.points().subspan(0, clipped.value())));
}

Result<double> iou(BBox box, PolygonView poly, ScratchBuffer& scratch) noexcept {
    if (!box.is_valid() || poly.size() < 3) {
        return Result<double>::err(GeomError::kDegenerateInput);
    }

    const auto corners = box.corners();
    const Result<double> intersection =
        polygon_intersection_area(PolygonView{corners}, poly, scratch);
    if (!intersection.ok()) {
        return intersection;
    }

    const double subject_area = box.area();
    const double clip_area = polygon_area(poly.vertices);
    const double union_area = subject_area + clip_area - intersection.value();
    if (union_area <= 0.0) {
        return Result<double>::ok(0.0);
    }
    return Result<double>::ok(intersection.value() / union_area);
}

Result<double> iou(PolygonView a, PolygonView b, ScratchBuffer& scratch) noexcept {
    if (a.size() < 3 || b.size() < 3) {
        return Result<double>::err(GeomError::kDegenerateInput);
    }

    const Result<double> intersection = polygon_intersection_area(a, b, scratch);
    if (!intersection.ok()) {
        return intersection;
    }

    const double union_area =
        polygon_area(a.vertices) + polygon_area(b.vertices) - intersection.value();
    if (union_area <= 0.0) {
        return Result<double>::ok(0.0);
    }
    return Result<double>::ok(intersection.value() / union_area);
}

Result<double> overlap_ratio(BBox box, PolygonView poly,
                             ScratchBuffer& scratch) noexcept {
    if (!box.is_valid() || poly.size() < 3) {
        return Result<double>::err(GeomError::kDegenerateInput);
    }

    const auto corners = box.corners();
    const Result<double> intersection =
        polygon_intersection_area(PolygonView{corners}, poly, scratch);
    if (!intersection.ok()) {
        return intersection;
    }

    const double subject_area = box.area();
    if (subject_area <= 0.0) {
        return Result<double>::ok(0.0);
    }
    return Result<double>::ok(intersection.value() / subject_area);
}

}  // namespace rge::spatial
