#ifndef PLANNER_OBSTACLE_H
#define PLANNER_OBSTACLE_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "carsize.h"
#include "typedef.h"
#include "coordinate_change.h"

/* 3个等级的障碍物大小 */
struct ObsSize {
 public:
  static ObsSize_type size_base;
  static ObsSize_type getsize(int n);

  static double vel_base;
  static double getvelocity(int n);

 private:
  static double increase_gras;  //障碍物等级梯度
};

/* obstacle class, include obstacle info */
/* include:
 * prime_pos ：primary pos 已经放大的
 * size_class ：障碍物大小等级
 * curr_pos ： 当前障碍物中心位置
 * path_set ： 障碍物路径信息
 * vel ： 障碍物velocity
 * yaw ： 障碍物航向角
 *  */
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(int size_class_, Point2d_type prime_pos_, Yaw_type yaw_);

  /* 移动到下一点 */
  void MoveStep(int row, int col);

  /* 计算当前这一段的长度与宽度 */
  ObsSize_type CurrentPartSize();

  int size_class;
  Point2d_type prime_pos;
  Yaw_type yaw;

  Point2d_type curr_pos;
  Point2d_type last_pos;
  std::vector<Point2d_type> path_set;
  double vel;
};

#endif