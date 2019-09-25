#ifndef PLANNER_COORDINATE_H
#define PLANNER_COORDINATE_H

/* 一些坐标转换操作需要的函数 */

#include <cmath>

#include "carsize.h"
#include "typedef.h"

/* opencv coordinate system
 * o ————x————>
 *   |
 *   y
 *   |
 *   v
 * */

/* gridmap coordinate system
 * o ————y————>
 *   |
 *   x
 *   |
 *   v
 * */

/* Gridmap to opencv draw */
/* int to double */
inline double GridmapToOpencvdraw(int value) {
  return static_cast<double>(value * OpencvdrawSize::edge_length() +
                             OpencvdrawSize::edge_length() / 2);
}
inline Point2d_type GridmapToOpencvdraw(const Point2i_type& pt) {
  return Point2d_type(GridmapToOpencvdraw(pt.x), GridmapToOpencvdraw(pt.y));
}

/* opencv draw to gridmap */
/* double to int */
inline int OpencvdrawToGridmap(double value) {
  return static_cast<int>(value / OpencvdrawSize::edge_length());
}
inline Point2i_type OpencvdrawToGridmap(const Point2d_type& pt) {
  return Point2i_type(OpencvdrawToGridmap(pt.x), OpencvdrawToGridmap(pt.y));
}

/* pt为待旋转的坐标
 * prime_pos 为原中心坐标
 * 返回引用
 *  */
inline void RotateCoordinate(const Point2d_type& pt, Yaw_type yaw,
                             Point2d_type& prime_pos) {
  prime_pos.x += pt.x * cos(yaw) - pt.y * sin(yaw);
  prime_pos.y += pt.x * sin(yaw) + pt.y * cos(yaw);
}

/* 重载坐标旋转函数
 * 输入与上相同，输出point type
 * */
inline Point2d_type RotateCoordinate(const Point2d_type& pt, Yaw_type yaw,
                                     const Point2d_type& prime_pos) {
  return Point2d_type(pt.x * cos(yaw) - pt.y * sin(yaw) + prime_pos.x,
                      pt.x * sin(yaw) + pt.y * cos(yaw) + prime_pos.y);
}

#endif