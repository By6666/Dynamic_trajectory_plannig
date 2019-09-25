#include "grid_input.h"

void GrideInput::GetOneGrid() {
  std::string file_name = FilePath + std::to_string(file_num_) + ".txt";

  // file_num_
  std::ifstream read_grid;
  read_grid.open(file_name, std::ios_base::in);
  if (!read_grid.is_open()) {
    std::cout << "file open failed !!" << std::endl;
    return;
  }

  read_grid >> rows_ >> columns_;

  std::string grid_buff = "";
  for (int i = 0; i < rows_ + 1; ++i) {
    getline(read_grid, grid_buff);  //整行读取
    //获得起点、终点、障碍物坐标
    for (int j = 0; j < grid_buff.length(); ++j) {
      if (grid_buff[j] == 's') {
        start_pos_.x = i - 1;
        start_pos_.y = j / 2;
      } else if (grid_buff[j] == 'g') {
        goal_pos_.x = i - 1;
        goal_pos_.y = j / 2;
      } else if (grid_buff[j] == 'x') {
        Point2i_type temp(i - 1, j / 2);
        obstacle_list_.push_back(CodeID(temp, columns_));
      }
    }
  }
  read_grid.close();
  start_yaw_ = JudgeStartYaw();
}

/* 判断起点车身yaw */
Yaw_type GrideInput::JudgeStartYaw() const {
  if (abs(goal_pos_.y - start_pos_.y) > abs(goal_pos_.x - start_pos_.x)) {
    if (goal_pos_.y > start_pos_.y)
      return M_PI_2;
    else
      return -M_PI_2;
  } else {
    if (goal_pos_.x > start_pos_.x)
      return 0.0;
    else
      return M_PI;
  }
}

//打印一张地图
void GrideInput::PrintMap() {
  std::cout << "primal map :" << std::endl;
  std::cout << "row: " << rows_ << "   col: " << columns_ << std::endl;
  std::cout << "start:" << start_pos_ << " goal:" << goal_pos_ << std::endl;
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < columns_; ++j) {
      if (start_pos_ == Point2i_type(i, j))
        std::cout << "s ";
      else if (goal_pos_ == Point2i_type(i, j))
        std::cout << "g ";
      else if (std::find(obstacle_list_.begin(), obstacle_list_.end(),
                         i * columns_ + j) != obstacle_list_.end())
        std::cout << "x ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
  // PrintObstacle();
}

void GrideInput::PrintObstacle() const {
  std::cout << "obstacle list : " << std::endl;
  for (auto &iter : obstacle_list_) {
    Point2i_type temp = DecodeID(iter, columns_);
    std::cout << "ID : " << iter << " | pos:" << temp << std::endl;
  }
}

// for testing
void GridInputOneMap(int num) {  //获得一个文本的map
  GrideInput grid_info(num);
  grid_info.GetOneGrid();
  //输出map的行列数
  std::cout << "rows:" << grid_info.get_grid_rows()
            << " columns:" << grid_info.get_grid_columns() << std::endl;
  //输出起点与终点坐标
  std::cout << "start_pos:" << grid_info.get_start_pos()
            << " goal_pos:" << grid_info.get_goal_pos() << std::endl;
  grid_info.PrintMap();
  grid_info.PrintObstacle();
}

void AllGridInput(int16_t scene_select_num) {  //输出全部的map
  for (int i = 0; i < kFile_Numbers; ++i) {    //获得所有文本的map
    GridInputOneMap(i + 1);
    std::cout << std::endl;
  }
}
