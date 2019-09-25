#ifndef PLANNER_BEZIER_CURVE_H
#define PLANNER_BEZIER_CURVE_H

#include <cmath>
#include <vector>

#include "state.h"
#include "typedef.h"

/* 三阶bezier curve */
void ThridOrderBezier(int pos_num, const std::vector<Point2d_type>& posset,
                      std::vector<DrawPathPos>& path);

/* 三阶bezier公式 */
template <class T = double>
inline T ThridOrderBezierFormula(T p0, T p1, T p2, T p3, T t) {
  return (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * pow(t, 3.0) +
         (3.0 * p0 - 6.0 * p1 + 3.0 * p2) * pow(t, 2.0) +
         (-3.0 * p0 + 3.0 * p1) * t + p0;
}

/* 三阶bezier 一阶导数公式 for yaw*/
template <class T = double>
inline T ThridOrderBezierDifferential(T p0, T p1, T p2, T p3, T t) {
  return (-3.0 * p0 + 9.0 * p1 - 9.0 * p2 + 3.0 * p3) * pow(t, 2.0) +
         (6.0 * p0 - 12.0 * p1 + 6.0 * p2) * t + (-3.0 * p0 + 3.0 * p1);
}

#endif