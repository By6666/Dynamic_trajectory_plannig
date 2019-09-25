#ifndef PLANNER_STATE_H
#define PLANNER_STATE_H

#include <cmath>

#include "typedef.h"

enum StateValue { NOT = false, IN = true };
/* key */
class KeyVaule {
 public:
  KeyVaule() = default;
  KeyVaule(double f_, double g_) : f(f_), g(g_) {}

  bool operator<(const KeyVaule& rg) const {
    return (f < rg.f) || (!(rg.f < f) && g < rg.g);
  }

  bool operator>(const KeyVaule& rg) const { return rg < *this; }
  double f;
  double g;
};

/* 点的位置信息 */
class CellPosState {
 public:
  CellPosState() = default;
  CellPosState(ID_SIZE id_, Yaw_type yaw_, int time_pos_ = -1)
      : id(id_), yaw(yaw_), time_pos(time_pos_) {}

  bool operator<(const CellPosState& other) const {
    return (id < other.id) || (!(other.id < id) && yaw < other.yaw);
  }

  bool operator==(const CellPosState& other) const {
    return (id == other.id) && (fabs(yaw - other.yaw) < DBL_EPSILON);
  }

  bool operator!=(const CellPosState& other) const { return !(*this == other); }

  ID_SIZE id;    // cell的id号
  Yaw_type yaw;  //航向角[-pi,pi]相对于地面
  int time_pos;
};

/* 存放f、g、h 三个值信息 */
struct Valude {
  double g;
  double h;
};

/* 每一个cell的信息，使用结构体定义 */
struct CellInfo {
  CellPosState posinfo;
  Valude value;
  bool inclose;
  CellPosState pre_best;
};

/* 加入openlist中比较的信息 */
struct CmpInfo {
  CellPosState posinfo;
  KeyVaule key;

  /* for Astar priority */
  bool operator>(const CmpInfo& pos) const {
    if (key.f == pos.key.f)
      return key.g < pos.key.g;
    else
      return key.f > pos.key.f;
  }
};

/* for opencv draw path */
class DrawPathPos {
 public:
  DrawPathPos() = default;
  DrawPathPos(const Point2d_type& pos, Yaw_type yaw_) : pt(pos), yaw(yaw_) {}

  Point2d_type pt;
  Yaw_type yaw;
};

/* print cellposinfo */
inline void PrintCellposState(const CellPosState& view, int col) {
  std::cout << DecodeID(view.id, col) << "  Yaw: " << view.yaw << std::endl;
}

#endif