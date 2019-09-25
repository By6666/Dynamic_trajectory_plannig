#include "obb.h"

/* 检测是否碰撞
 * 输入：无
 * 输出：true 碰撞; false 不碰撞
 * */
bool OBB::IsCollision() {
  /* 获得所有投影轴 */
  get_all_projection_axis();

  /* 对象在投影轴上的投影范围 */
  for (auto& elem : projection_axis_) {
    if (!JudgeIntersecting(CalculateRange(elem, obj_1_),
                           CalculateRange(elem, obj_2_)))
      return false;
  }
  return true;
}

/* 获得所有投影轴 */
void OBB::get_all_projection_axis() {
  for (int16_t i = 0; i < obj_1_.size(); ++i) {
    projection_axis_.push_back(get_single_projection_axis(
        CalculateVector(obj_1_[i], obj_1_[(i + 1) % obj_1_.size()])));
  }
  for (int16_t i = 0; i < obj_2_.size(); ++i) {
    projection_axis_.push_back(get_single_projection_axis(
        CalculateVector(obj_2_[i], obj_2_[(i + 1) % obj_2_.size()])));
  }
}

/* 获得单个投影轴
 * 输入：待检测轴的向量
 * 输出：输入轴的单位法相量
 * */
Point_type OBB::get_single_projection_axis(const Point_type& check_edge) {
  double length = sqrtf(powf(check_edge.x, 2.0f) + powf(check_edge.y, 2.0f));
  Point_type result;
  result.x = check_edge.y / length;
  result.y = (0.0f - check_edge.x) / length;

  return result;
}

/* 计算一个obj在投影轴上的范围
 * 输入：投影轴，被检测对象
 * 输入：输入对象在输入投影轴上的范围
 * */
OBB::ResultRange OBB::CalculateRange(const Point_type& project_axis,
                                     const PointSet_type& obj) {
  double min = std::numeric_limits<double>::max();
  double max = -std::numeric_limits<double>::max();

  for (auto& elem : obj) {
    double temp = ScalarProduct(project_axis, elem);
    if (temp < min) min = temp;
    if (temp > max) max = temp;
  }

  return ResultRange{min, max};
}
