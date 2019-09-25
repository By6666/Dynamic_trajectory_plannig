#ifndef PLANNER_PATH_PROCESS_H
#define PLANNER_PATH_PROCESS_H

#include <list>
#include <vector>

#include "bezier_curve.h"
#include "opencvdraw.h"
#include "state.h"
#include "typedef.h"

#define PATHDIVID 7  //插值个数

/* swap x & y for opencv draw*/
inline Point2d_type SwapPointXY(const Point2d_type& pt) {
  return Point2d_type(pt.y, pt.x);
}

/* path smooth */
bool PathSmooth(int col, const std::list<CellPosState>& path,
                std::vector<DrawPathPos>& final_path);

/* 直线路段 插值 */
void PathInterpoletion(const DrawPathPos& start, const DrawPathPos& end,
                       int size, int pre_part_num,
                       std::vector<DrawPathPos>& final_path);

#endif