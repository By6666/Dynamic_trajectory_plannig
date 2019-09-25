#ifndef PLANNER_EXPANDDEF_H
#define PLANNER_EXPANDDEF_H

#include "typedef.h"

/* 扩展点顺序如下
   ---------------------
   |   |1 5|   |1 6|   |
   ---------------------
   |1 4| 8 | 1 | 5 | 9 |
   ---------------------
   |   | 3 | 0 | 4 |   |
   ---------------------
   |1 3| 7 | 2 | 6 |1 0|
   ---------------------
   |   |1 2|   |1 1|   |
   ---------------------
*/
enum ExpandState {
  Up = 0b1,
  Dw = 0b10,
  Lf = 0b100,
  Rg = 0b1000,

  Up_Rg = 0b10000,
  Dw_Rg = 0b100000,
  Dw_Lf = 0b1000000,
  Up_Lf = 0b10000000,

  Rg_Up_26_5 = 0b100000000,
  Rg_Dw_26_5 = 0b1000000000,
  Dw_Rg_26_5 = 0b10000000000,
  Dw_Lf_26_5 = 0b100000000000,
  Lf_Dw_26_5 = 0b1000000000000,
  Lf_Up_26_5 = 0b10000000000000,
  Up_Lf_26_5 = 0b100000000000000,
  Up_Rg_26_5 = 0b1000000000000000,
};
namespace ExpandPos {
/* 1 */
inline Point2i_type Up(const Point2i_type& pos) {
  return Point2i_type(pos.x - 1, pos.y);
}
/* 2 */
inline Point2i_type Dw(const Point2i_type& pos) {
  return Point2i_type(pos.x + 1, pos.y);
}
/* 3 */
inline Point2i_type Lf(const Point2i_type& pos) {
  return Point2i_type(pos.x, pos.y - 1);
}
/* 4 */
inline Point2i_type Rg(const Point2i_type& pos) {
  return Point2i_type(pos.x, pos.y + 1);
}
/* 5 */
inline Point2i_type Up_Rg(const Point2i_type& pos) {
  return Point2i_type(pos.x - 1, pos.y + 1);
}
/* 6 */
inline Point2i_type Dw_Rg(const Point2i_type& pos) {
  return Point2i_type(pos.x + 1, pos.y + 1);
}
/* 7 */
inline Point2i_type Dw_Lf(const Point2i_type& pos) {
  return Point2i_type(pos.x + 1, pos.y - 1);
}
/* 8 */
inline Point2i_type Up_Lf(const Point2i_type& pos) {
  return Point2i_type(pos.x - 1, pos.y - 1);
}
/* 9 */
inline Point2i_type Rg_Up_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x - 1, pos.y + 2);
}
/* 10 */
inline Point2i_type Rg_Dw_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x + 1, pos.y + 2);
}
/* 11 */
inline Point2i_type Dw_Rg_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x + 2, pos.y + 1);
}
/* 12 */
inline Point2i_type Dw_Lf_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x + 2, pos.y - 1);
}
/* 13 */
inline Point2i_type Lf_Dw_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x + 1, pos.y - 2);
}
/* 14 */
inline Point2i_type Lf_Up_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x - 1, pos.y - 2);
}
/* 15 */
inline Point2i_type Up_Lf_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x - 2, pos.y - 1);
}
/* 16 */
inline Point2i_type Up_Rg_26_5(const Point2i_type& pos) {
  return Point2i_type(pos.x - 2, pos.y + 1);
}
}  // namespace ExpandPos

#endif