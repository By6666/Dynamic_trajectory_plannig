/* 生成障碍物信息函数的头文件 */

#ifndef PLANNER_OBS_CREATE_H
#define PLANNER_OBS_CREATE_H

#include <algorithm>
#include <set>
#include <vector>

#include "coordinate_change.h"
#include "grid_input.h"
#include "obstacle.h"
#include "typedef.h"

#define ObsPathNumUpper 100

/* 生成obs对象 */
void GenerateObstacle(std::vector<ID_SIZE>& obs, int row, int col, Yaw_type yaw,
                      std::vector<Obstacle>& obstacle);

/* 障碍物路径预测 */
void ObstaclePathPrediction(std::vector<Obstacle>& obstacle, int row, int col);

/* 找到障碍物的中心坐标 */
Point2d_type GetObsCentralPos(ID_SIZE id, int size_class, int col,
                              Yaw_type yaw);

/* 获得障碍物的下一个点的ID */
ID_SIZE GetNextID(const Point2i_type& pt, Yaw_type yaw, int col);

/* 是否在vector中 */
template <class T = ID_SIZE>
inline bool IsInVector(const std::vector<T>& vector, T id) {
  return std::find(vector.begin(), vector.end(), id) != vector.end();
}

/* 是否在set中 */
template <class T = ID_SIZE>
inline bool IsInSet(const std::set<T>& set, T id) {
  return set.find(id) != set.end();
}

/* 是否在map中 */
inline bool IsInGridmap(int row, int col, Point2d_type pt) {
  return OpencvdrawToGridmap(pt.x) >= 0 && OpencvdrawToGridmap(pt.x) < row &&
         OpencvdrawToGridmap(pt.y) >= 0 && OpencvdrawToGridmap(pt.y) < col;
}

#endif