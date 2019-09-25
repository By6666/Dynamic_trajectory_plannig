#ifndef PLANNER_ASTAR_H
#define PLANNER_ASTAR_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "carsize.h"
#include "collision_checking.h"
#include "expanddef.h"
#include "obs_create.h"
#include "state.h"
#include "typedef.h"

/* Astar class */
class Astar {
 public:
  Astar(int row, int columns, Point2i_type start, Point2i_type goal,
        Yaw_type start_yaw, const std::vector<Obstacle>& dynamic_obs,
        int time_begin_);
  Astar(const Astar&) = delete;

  /* for openlist */
  typedef std::priority_queue<CmpInfo, std::vector<CmpInfo>,
                              std::greater<CmpInfo> >
      priority_que;

  /* init */
  void Init();

  /* compute shortest path */
  bool ComputePath();

  /* 计算h值 */
  inline double GetHeuristicValue(const Point2i_type& pt) const {
    // 切比雪夫距离
    // return std::max(fabs(pt.x - goal_pos_.x), fabs(pt.y - goal_pos_.y));

    //Euler
    return TwoPosDistence(pt, goal_pos_);
  }

  /* 获得neighbor */
  std::vector<CellPosState> GetNeighbor(const CellPosState& pos);

  /* 路径回溯 */
  void RecurPath();

  /* 清空上一次搜索时的素据 */
  void ClearCache();

  /* 执行Astar */
  bool Execute();

  /* print result */
  void PrintResult();

  /**** 获得类内成员函数 ****/
  inline int get_row() const { return row_; }
  inline int get_col() const { return col_; }
  inline const Point2i_type& get_start() const { return start_pos_; }
  inline const Point2i_type& get_goal() const { return goal_pos_; }
  inline const std::list<CellPosState>& get_final_path() const {
    return final_path_;
  }
  inline std::vector<ID_SIZE> get_point_path() const { return path_; }
  inline const std::vector<Obstacle>& get_obs_info() const {
    return dynamic_obs_;
  }
  inline Yaw_type get_start_yaw() const { return start_yaw_; }

  /**** 设置类内成员函数 ****/
  inline int& set_time_begin() { return time_begin; }

 private:
  int row_, col_;
  int time_begin;
  Point2i_type start_pos_, goal_pos_;
  CellInfo *start_, *goal_;
  Yaw_type start_yaw_;
  std::list<CellPosState> final_path_;
  std::vector<ID_SIZE> path_;
  std::map<CellPosState, CellInfo> all_cell_info_;
  const std::vector<Obstacle>& dynamic_obs_;  //障碍物信息列表

  int expand_cnt_;

  /* 修改cellinfo中isclose信息 */
  inline void SetStateValue(CellInfo* const cell, bool state) {
    cell->inclose = state;
  }

  /* 计算KeyValue */
  inline KeyVaule CalculateKey(const CellInfo* const ptr) const {
    double f = (ptr->value.g + ptr->value.h) == DBL_MAX
                   ? DBL_MAX
                   : (ptr->value.g + ptr->value.h);
    return KeyVaule(f, ptr->value.g);
  }

  /* 计算KeyValue overload*/
  inline KeyVaule CalculateKey(double g, double h) const {
    double f = (g + h) == DBL_MAX ? DBL_MAX : (g + h);
    return KeyVaule(f, g);
  }

  /* 判断是否在界内 */
  inline bool IsInMapBoundry(const Point2i_type& pos) const {
    return pos.x >= 0 && pos.x < row_ && pos.y >= 0 && pos.y < col_;
  }

  /* 针对26.5度方向的扩展点 */
  template <class T = Point2i_type>
  void NeiborsPush_26_5(std::vector<CellPosState>& neighbors, const T& pos,
                        T (*first)(const T&), T (*second)(const T&), int time);
  /* 针对垂直方向 */
  bool NeiborsPush(std::vector<CellPosState>& neighbors,
                   const Point2i_type& pos,
                   Point2i_type (*first)(const Point2i_type&), Yaw_type yaw,
                   int time);

  /* 两点间距离 Euler */
  inline double TwoPosDistence(const Point2i_type& pt1,
                               const Point2i_type& pt2) const {
    return sqrt(pow((pt1.x - pt2.x), 2.0) + pow((pt1.y - pt2.y), 2.0));
  }

  /* 两点间距离 overload */
  inline double TwoPosDistence(ID_SIZE id1, ID_SIZE id2) {
    return TwoPosDistence(DecodeID(id1, col_), DecodeID(id2, col_));
  }

  /* push openlist */
  bool PushOpenlist(const CellInfo* const curr_cell,
                    const CellPosState& neighbor, priority_que& openlist);

  /* Is in path list */
  inline bool IsInPath(ID_SIZE id) {
    return std::find(path_.begin(), path_.end(), id) != path_.end();
  }

  inline void PrintPosState(const CellPosState& view) {
    PrintCellposState(view, col_);
  }
};

#endif