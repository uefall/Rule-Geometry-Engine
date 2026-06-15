#pragma once

#include "rge/coordinate_system.hpp"
#include "rge/error.hpp"
#include "rge/scratch_buffer.hpp"

#include "rge/geom/bbox.hpp"
#include "rge/geom/line.hpp"
#include "rge/geom/point.hpp"
#include "rge/geom/polygon_view.hpp"

#include "rge/spatial/contains.hpp"
#include "rge/spatial/distance.hpp"
#include "rge/spatial/intersects.hpp"
#include "rge/spatial/iou.hpp"
#include "rge/spatial/polygon_ops.hpp"
#include "rge/spatial/side_of_line.hpp"

#include "rge/trajectory/fixed_ring_buffer.hpp"
#include "rge/trajectory/track_cross_line.hpp"
#include "rge/trajectory/track_kinematics.hpp"
#include "rge/trajectory/track_pool.hpp"
#include "rge/trajectory/track_relation.hpp"
#include "rge/trajectory/track_state.hpp"
#include "rge/trajectory/track_update.hpp"

#include "rge/rules/events.hpp"
#include "rge/rules/rule_context.hpp"
