#include "Astar.h"
/* 针对26.5度方向的扩展点
 * first:26.5度方向，second:45度方向
 * */
template <class T>
inline void Astar::NeiborsPush_26_5(std::vector<CellPosState>& neighbors,
                                    const T& pos, T (*first)(const T&),
                                    T (*second)(const T&), int time) {
  T temp = first(pos);
  if (IsInMapBoundry(temp)) {
    neighbors.push_back(
        CellPosState(CodeID(temp, col_), calculate_arct2(pos, temp), time + 1));
  }
}

/* 针对垂直方向 */
inline bool Astar::NeiborsPush(std::vector<CellPosState>& neighbors,
                               const Point2i_type& pos,
                               Point2i_type (*first)(const Point2i_type&),
                               Yaw_type yaw, int time) {
  Point2i_type temp = first(pos);
  if (IsInMapBoundry(temp)) {
    neighbors.push_back(CellPosState(CodeID(temp, col_), yaw, time + 1));
    return true;
  } else {
    return false;
  }
}

/* 获得neighbor */
std::vector<CellPosState> Astar::GetNeighbor(const CellPosState& pos) {
  std::vector<CellPosState> neighbors;

  Point2i_type curr_pos = DecodeID(pos.id, col_);
  Yaw_type yaw = pos.yaw;
  int time_pos = pos.time_pos;
  int state_flg = 0;

  /* 起点的下一点只能为同向 */
  if (curr_pos == start_pos_) {
    state_flg = -1;
    Point2i_type temp = ExpandPos::Dw(Point2i_type(0, 0));
    Point2i_type neig;
    neig.x = temp.x * cos(yaw) - temp.y * sin(yaw) + curr_pos.x;
    neig.y = temp.x * sin(yaw) + temp.y * cos(yaw) + curr_pos.y;

    if (IsInMapBoundry(neig)) {
      neighbors.push_back(CellPosState(CodeID(neig, col_), yaw, time_pos + 1));
    }
  }

  while (!state_flg) {
    /* Down */
    if (state_flg == 0 && TwoDoubleEqual(yaw, 0.0)) {
      state_flg = ExpandState::Dw;
      if (!NeiborsPush(neighbors, curr_pos, ExpandPos::Dw, 0.0, time_pos))
        break;

      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Dw_Rg_26_5,
                       ExpandPos::Dw_Rg, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Dw_Lf_26_5,
                       ExpandPos::Dw_Lf, time_pos);
      break;
    }

    /* Right */
    if (state_flg == 0 && TwoDoubleEqual(yaw, M_PI_2)) {
      state_flg = ExpandState::Rg;
      if (!NeiborsPush(neighbors, curr_pos, ExpandPos::Rg, M_PI_2, time_pos))
        break;

      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Rg_Dw_26_5,
                       ExpandPos::Dw_Rg, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Rg_Up_26_5,
                       ExpandPos::Up_Rg, time_pos);
      break;
    }

    /* left */
    if (state_flg == 0 && TwoDoubleEqual(yaw, -M_PI_2)) {
      state_flg = ExpandState::Lf;
      if (!NeiborsPush(neighbors, curr_pos, ExpandPos::Lf, -M_PI_2, time_pos))
        break;

      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Lf_Dw_26_5,
                       ExpandPos::Dw_Lf, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Lf_Up_26_5,
                       ExpandPos::Up_Lf, time_pos);
      break;
    }

    /* Up */
    if (state_flg == 0 &&
        (TwoDoubleEqual(yaw, M_PI) || TwoDoubleEqual(yaw, -M_PI))) {
      state_flg = ExpandState::Up;
      if (!NeiborsPush(neighbors, curr_pos, ExpandPos::Up, M_PI, time_pos))
        break;

      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Up_Lf_26_5,
                       ExpandPos::Up_Lf, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Up_Rg_26_5,
                       ExpandPos::Up_Rg, time_pos);
      break;
    }

    /* Dw_Rg_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, 0.0, 0.0 + CarSize::kConstraintValue())) {
      state_flg = ExpandState::Dw_Rg_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Dw, 0.0, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Dw_Rg_26_5,
                       ExpandPos::Dw_Rg, time_pos);
      break;
    }

    /* Dw_Lf_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, 0.0, 0.0 - CarSize::kConstraintValue())) {
      state_flg = ExpandState::Dw_Lf_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Dw, 0.0, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Dw_Lf_26_5,
                       ExpandPos::Dw_Lf, time_pos);
      break;
    }

    /* Rg_Up_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, M_PI_2, M_PI_2 + CarSize::kConstraintValue())) {
      state_flg = ExpandState::Rg_Up_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Rg, M_PI_2, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Rg_Up_26_5,
                       ExpandPos::Up_Rg, time_pos);
      break;
    }

    /* Rg_Dw_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, M_PI_2, M_PI_2 - CarSize::kConstraintValue())) {
      state_flg = ExpandState::Rg_Dw_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Rg, M_PI_2, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Rg_Dw_26_5,
                       ExpandPos::Dw_Rg, time_pos);
      break;
    }

    /* Lf_Dw_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, -M_PI_2, -M_PI_2 + CarSize::kConstraintValue())) {
      state_flg = ExpandState::Lf_Dw_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Lf, -M_PI_2, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Lf_Dw_26_5,
                       ExpandPos::Dw_Lf, time_pos);
      break;
    }

    /* Lf_Up_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, -M_PI_2, -M_PI_2 - CarSize::kConstraintValue())) {
      state_flg = ExpandState::Lf_Up_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Lf, -M_PI_2, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Lf_Up_26_5,
                       ExpandPos::Up_Lf, time_pos);
      break;
    }

    /* Up_Rg_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, M_PI, M_PI - CarSize::kConstraintValue())) {
      state_flg = ExpandState::Up_Rg_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Up, M_PI, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Up_Rg_26_5,
                       ExpandPos::Up_Rg, time_pos);
      break;
    }

    /* Up_Lf_26.5 */
    if (state_flg == 0 &&
        YawInRange(yaw, -M_PI, -M_PI + CarSize::kConstraintValue())) {
      state_flg = ExpandState::Up_Lf_26_5;
      NeiborsPush(neighbors, curr_pos, ExpandPos::Up, M_PI, time_pos);
      NeiborsPush_26_5(neighbors, curr_pos, ExpandPos::Up_Lf_26_5,
                       ExpandPos::Up_Lf, time_pos);
      break;
    }
  }

  /* 碰撞检测 */
  std::vector<CellPosState> final_neighbor;
  for (auto& elem : neighbors) {
    if (CollisionChecking(col_, time_pos, curr_pos, DecodeID(elem.id, col_),
                          elem.yaw, dynamic_obs_)) {
      continue;
    } else {
      final_neighbor.push_back(elem);
    }
  }
  return final_neighbor;
}
