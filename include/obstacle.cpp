#include "obstacle.h"

/* 障碍物基础大小 */
ObsSize_type ObsSize::size_base = ObsSize_type(30.0, 20.0);
double ObsSize::increase_gras = 16;

/* 获得障碍物等级对应的大小 */
ObsSize_type ObsSize::getsize(int n) {
  return ObsSize_type(size_base.width + increase_gras * n, size_base.height);
}

/* 障碍物基础速度大小  单位 m/s */
double ObsSize::vel_base = 30.0 / 3.6;
double ObsSize::getvelocity(int n) { return vel_base / n; }

Obstacle::Obstacle(int size_class_, Point2d_type prime_pos_, Yaw_type yaw_)
    : size_class(size_class_), prime_pos(prime_pos_), yaw(yaw_) {
  /* init  */
  last_pos = curr_pos = prime_pos;
  path_set.push_back(curr_pos);

  vel = ObsSize::getvelocity(size_class) * 5.0;
}

/* 移动到下一点 */
void Obstacle::MoveStep(int row, int col) {
  last_pos = curr_pos;

  /* obs 轨迹预测 */
  curr_pos.x = path_set.back().x + vel * cos(yaw);
  curr_pos.y = path_set.back().y + vel * sin(yaw);

  // RotateCoordinate(Point2d_type(5.0, 0.0), yaw, curr_pos);

  path_set.push_back(curr_pos);
}

/* 计算当前这一段的长度与宽度 */
ObsSize_type Obstacle::CurrentPartSize() {
  double width = 0.0, height = 0.0;
  width = sqrt(pow(curr_pos.x - last_pos.x, 2.0) +
               pow(curr_pos.y - last_pos.y, 2.0)) +
          ObsSize::getsize(size_class).width;

  height = ObsSize::getsize(size_class).height;
  return ObsSize_type(width, height);
}