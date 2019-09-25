#ifndef PLANNER_COLLISION_H
#define PLANNER_COLLISION_H

#include <vector>

#include "carsize.h"
#include "obstacle.h"
#include "typedef.h"

/* class of collision checking obj
 * inculding:
 * central pos and frame body of obj,
 * max_dis:distence beteween obj central and frame body farthest vertex
 * min_dis:half width of obj
 * */
class CollisionObj {
 public:
  CollisionObj() = default;
  CollisionObj(const Point2d_type& central_,
               const std::vector<Point2d_type>& frame_, double max_,
               double min_)
      : central(central_), frame(frame_), max_dis(max_), min_dis(min_) {}

  Point2d_type central;
  std::vector<Point2d_type> frame;
  double max_dis, min_dis;
};

#endif