#include <iostream>

#include "rge/rge.hpp"

int main() {
    rge::ScratchBuffer scratch;

    const rge::BBox box_a = rge::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
    const rge::BBox box_b = rge::BBox::from_xywh(5.0, 5.0, 10.0, 10.0);
    const auto iou = rge::spatial::iou(box_a, box_b);
    std::cout << "bbox IoU = " << iou.value() << '\n';

    rge::trajectory::TrackState<16> track{};
    rge::trajectory::TrackConfig config{};
    rge::trajectory::update(track,
                            rge::trajectory::TrackObservation{1, 0, box_a},
                            config);
    rge::trajectory::update(track,
                            rge::trajectory::TrackObservation{1, 1,
                                                                rge::BBox::from_xywh(1, 0, 10, 10)},
                            config);
    std::cout << "track speed = " << rge::trajectory::speed(track) << '\n';
    return 0;
}
