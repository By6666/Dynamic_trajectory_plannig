#ifndef PLANNER_COLLISION_CHECKING_H
#define PLANNER_COLLISION_CHECKING_H

#include <cmath>
#include <iostream>
#include <vector>

#include "carsize.h"
#include "collision_obj.h"
#include "coordinate_change.h"
#include "obb.h"
#include "obstacle.h"
#include "typedef.h"

/* get obj all vertex */
std::vector<Point2d_type> GetBorder(const Point2d_type& central, Yaw_type yaw,
                                    double frame_length, double frame_width);

/* rotate obj */
void RotateObj(std::vector<Point2d_type>& poset,
               const Point2d_type& central_point, Yaw_type yaw);

/* collision checking */
bool CollisionChecking(int col, int time, const Point2i_type& first_pos,
                       const Point2i_type& second_pos, Yaw_type yaw,
                       const std::vector<Obstacle>& obstacle_list);

/* collision checking between two obj */
bool TwoObjCollisionChecking(const CollisionObj& obj1,
                             const CollisionObj& obj2);

/* two pos Euler dis */
inline double TwoPosDis(const Point2d_type& pt1, const Point2d_type& pt2) {
  return sqrt(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0));
}

/* create obj for checking  */
CollisionObj CreareCollisionObj(Point2d_type start, Point2d_type end,
                                double obj_length, double obj_width,
                                Yaw_type yaw);

#endif