#ifndef PLANNER_GRID_INPUT_H
#define PLANNER_GRID_INPUT_H

#include <stdint.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

#include "typedef.h"

/* gridmap coordinate system
 * o ————y————>
 *   |
 *   x
 *   |
 *   v
 * */

const int kFile_Numbers = 960;  //需要读取的文件数目
const char FilePath[] =
    "//home//by//Documents//Planner//dynamic_avoidance_obstacle//highway//";

class GrideInput {
 public:
  GrideInput(int file_number) : file_num_(file_number) {}

  void GetOneGrid();  //得到一张地图
  void PrintMap();    //打印map

  Yaw_type JudgeStartYaw() const;

  inline int get_grid_rows() const { return rows_; }
  inline int get_grid_columns() const { return columns_; }
  inline Point2i_type get_start_pos() const { return start_pos_; }
  inline Point2i_type get_goal_pos() const { return goal_pos_; }
  inline const std::vector<ID_SIZE>& get_obstacle_pos() const {
    return obstacle_list_;
  }
  inline std::vector<ID_SIZE>& get_obstacle_pos() { return obstacle_list_; }
  inline Yaw_type get_start_yaw() const { return start_yaw_; }
  void PrintObstacle() const;

 private:
  int rows_, columns_, file_num_;
  Yaw_type start_yaw_;
  int16_t scene_flg_;
  Point2i_type start_pos_, goal_pos_;
  std::vector<ID_SIZE> obstacle_list_;
};

void GridInputOneMap(int num);                //获得一个文本的map
void AllGridInput(int16_t scene_select_num);  //输出全部的map

#endif