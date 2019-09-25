/*
 * OBB碰撞检测算法(基于分离轴定理)
 */
#ifndef OBB_H
#define OBB_H

#include <iostream>
#include <limits>
#include <vector>

#include "typedef.h"

typedef Point2d_type Point_type;
typedef std::vector<Point_type> PointSet_type;

/* OBB检测类
 * 输入：两个多边形对象
 * 一个公共成员函数
 * */

class OBB {
 public:
  /* 构造函数 */
  OBB(const PointSet_type& object_l, const PointSet_type& object_r)
      : obj_1_(object_l), obj_2_(object_r) {
    projection_axis_.reserve(obj_1_.size() + obj_2_.size());
  }

  /* 检测是否碰撞 */
  bool IsCollision();

 private:
  /* 一个表示范围的结构体 */
  struct ResultRange {
    double min;
    double max;
  };
  PointSet_type obj_1_;            //对象1
  PointSet_type obj_2_;            //对象2
  PointSet_type projection_axis_;  //所有投影轴的容器

  /* 获得所有投影轴 */
  void get_all_projection_axis();

  /* 获得单个投影轴 */
  Point_type get_single_projection_axis(const Point_type& check_edge);

  /* 计算一个obj在投影轴上的范围 */
  ResultRange CalculateRange(const Point_type& project_axis,
                             const PointSet_type& obj);

  /* 向量 vertex_2 指向 vertex_1 */
  inline Point_type CalculateVector(const Point_type& vertex_1,
                                    const Point_type& vertex_2) {
    return Point_type(vertex_1.x - vertex_2.x, vertex_1.y - vertex_2.y);
  }

  /* 两个向量的点乘 */
  inline double ScalarProduct(const Point_type& vector_1,
                              const Point_type& vector_2) {
    return vector_1.x * vector_2.x + vector_1.y * vector_2.y;
  }

  /* 判断两个范围有没有交集
   * true 相交，false 不相交
   * */
  inline bool JudgeIntersecting(const ResultRange& rang_1,
                                const ResultRange& rang_2) {
    return rang_1.max >= rang_2.min && rang_2.max >= rang_1.min;
  }
};

#endif
