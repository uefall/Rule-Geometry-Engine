#include "rge/geom/polygon_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rge {

double polygon_signed_area(std::span<const Point2> vertices) {
    if (vertices.size() < 3) {
        return 0.0;
    }
    double area = 0.0;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const std::size_t j = (i + 1) % vertices.size();
        area += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
    }
    return area * 0.5;
}

BBox polygon_bounding_box(std::span<const Point2> vertices) {
    if (vertices.empty()) {
        return {};
    }
    double min_x = vertices[0].x;
    double min_y = vertices[0].y;
    double max_x = vertices[0].x;
    double max_y = vertices[0].y;
    for (const Point2& vertex : vertices) {
        min_x = std::min(min_x, vertex.x);
        min_y = std::min(min_y, vertex.y);
        max_x = std::max(max_x, vertex.x);
        max_y = std::max(max_y, vertex.y);
    }
    return BBox{{min_x, min_y}, {max_x, max_y}};
}

double PolygonView::signed_area() const { return polygon_signed_area(vertices); }

BBox PolygonView::bounding_box() const { return polygon_bounding_box(vertices); }

}  // namespace rge
