#include "Astar.h"

/* Astar 构造函数
 * input: row、column、start、goal、start yaw、obstacle info、time begin sit
 * */
Astar::Astar(int row, int columns, Point2i_type start, Point2i_type goal,
             Yaw_type start_yaw, const std::vector<Obstacle>& dynamic_obs,
             int time_begin_)
    : row_(row),
      col_(columns),
      start_pos_(start),
      goal_pos_(goal),
      start_yaw_(start_yaw),
      dynamic_obs_(dynamic_obs),
      time_begin(time_begin_) {
  expand_cnt_ = 0;
}

/* init search info
 * */
void Astar::Init() {
  all_cell_info_[CellPosState(CodeID(start_pos_, col_), start_yaw_,
                              time_begin)] = {
      CellPosState(CodeID(start_pos_, col_), start_yaw_, time_begin),
      {0.0, GetHeuristicValue(start_pos_)},
      StateValue::NOT,
      CellPosState(-1, -1)};

  all_cell_info_[CellPosState(CodeID(goal_pos_, col_), start_yaw_)] = {
      CellPosState(CodeID(goal_pos_, col_), start_yaw_),
      {DBL_MAX, 0},
      StateValue::NOT,
      CellPosState(-1, -1)};

  start_ = &all_cell_info_[CellPosState(CodeID(start_pos_, col_), start_yaw_)];
  goal_ = &all_cell_info_[CellPosState(CodeID(goal_pos_, col_), start_yaw_)];
}

/* compute shortest path
 * output: true search sucessed; false failed
 * */
bool Astar::ComputePath() {
  /* init */
  Init();
  priority_que openlist;
  openlist.push({start_->posinfo, CalculateKey(start_)});  // push start

  while (!openlist.empty()) {
    CellInfo* cur_cell = NULL;
    cur_cell = &all_cell_info_[openlist.top().posinfo];
    openlist.pop();

    if (cur_cell->posinfo == goal_->posinfo) {
      return true;
    }
    if (cur_cell->inclose == StateValue::NOT) {
      SetStateValue(cur_cell, StateValue::IN);

      /* get neighbors */
      std::vector<CellPosState> neighbors = GetNeighbor(cur_cell->posinfo);
      int neighbors_cnt = 0;  // for expand_cnt_
      for (auto& elem : neighbors) {
        if (PushOpenlist(cur_cell, elem, openlist)) ++neighbors_cnt;
      }
      if (neighbors_cnt) ++expand_cnt_;
    }
  }
  return false;
}

/* push openlist
 * output: true:there is pos pushed openlist & false nothing pushed
 * */
bool Astar::PushOpenlist(const CellInfo* const cur_cell,
                         const CellPosState& neighbor, priority_que& openlist) {
  bool result = false;
  auto iter = all_cell_info_.find(neighbor);
  double g =
      cur_cell->value.g + TwoPosDistence(cur_cell->posinfo.id, neighbor.id);
  double h = GetHeuristicValue(DecodeID(neighbor.id, col_));

  if (iter == all_cell_info_.end()) {
    all_cell_info_[neighbor] =
        CellInfo{neighbor, {g, h}, StateValue::NOT, cur_cell->posinfo};

    openlist.push(CmpInfo{neighbor, CalculateKey(g, h)});
    result = true;
  } else {
    if (iter->second.inclose == StateValue::NOT) {
      iter->second.value.g = g;
      iter->second.value.h = h;
      iter->second.pre_best = cur_cell->posinfo;
      openlist.push(CmpInfo{neighbor, CalculateKey(g, h)});
      result = true;
    }
  }
  return result;
}

/* path recurent for final path
 * */
void Astar::RecurPath() {
  CellPosState node = goal_->posinfo;
  final_path_.push_front(node);

  while (node != start_->posinfo) {
    node = all_cell_info_[node].pre_best;
    final_path_.push_front(node);
    path_.push_back(node.id);
  }
  for (int i = 0; i < time_begin; ++i) {
    final_path_.push_front(node);
  }
}

/* clear storage container for computing path */
void Astar::ClearCache() {
  all_cell_info_.clear();
  path_.clear();
  final_path_.clear();
}

/* execute Astar */
bool Astar::Execute() {
  ClearCache();
  if (ComputePath()) {
    RecurPath();
    return true;
  }
  return false;
}

/* print result */
void Astar::PrintResult() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < col_; ++j) {
      if (start_pos_.x == i && start_pos_.y == j)
        std::cout << "s ";

      else if (goal_pos_.x == i && goal_pos_.y == j)
        std::cout << "g ";

      else if (IsInPath(i * col_ + j))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "search times : " << time_begin + 1
            << "    shortest path step nums : " << path_.size()
            << "    expand point nums : " << expand_cnt_ << std::endl;

  std::cout << std::endl << std::endl;
}