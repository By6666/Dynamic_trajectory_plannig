#ifndef PLANNER_TYPEDEF_H
#define PLANNER_TYPEDEF_H

#include <opencv2/opencv.hpp>  //for Point type

#define M_E 2.7182818284590452354         /* e */
#define M_LOG2E 1.4426950408889634074     /* log_2 e */
#define M_LOG10E 0.43429448190325182765   /* log_10 e */
#define M_LN2 0.69314718055994530942      /* log_e 2 */
#define M_LN10 2.30258509299404568402     /* log_e 10 */
#define M_PI 3.14159265358979323846       /* pi */
#define M_PI_2 1.57079632679489661923     /* pi/2 */
#define M_PI_4 0.78539816339744830962     /* pi/4 */
#define M_1_PI 0.31830988618379067154     /* 1/pi */
#define M_2_PI 0.63661977236758134308     /* 2/pi */
#define M_2_SQRTPI 1.12837916709551257390 /* 2/sqrt(pi) */
#define M_SQRT2 1.41421356237309504880    /* sqrt(2) */
#define M_SQRT1_2 0.70710678118654752440  /* 1/sqrt(2) */

typedef int ID_SIZE;
typedef double Yaw_type;
typedef cv::Point2i Point2i_type;
typedef cv::Point2f Point2f_type;
typedef cv::Point2d Point2d_type;
typedef cv::Size2d ObsSize_type;

/* ID编码 */
inline ID_SIZE CodeID(const Point2i_type& xoy, int col) {
  return static_cast<ID_SIZE>(xoy.x * col + xoy.y);
}
/* ID解码 */
inline Point2i_type DecodeID(ID_SIZE id, int col) {
  return Point2i_type(id / col, id % col);
}
/* 计算两点连线的斜率 */
inline double calculate_arct2(const Point2i_type& pos1,
                              const Point2i_type& pos2) {
  return atan2((pos2.y - pos1.y), (pos2.x - pos1.x));
}
/* 判断两个double是否相等 */
inline bool TwoDoubleEqual(double lf, double rg) {
  return fabs(lf - rg) < DBL_EPSILON;
}
/* 判断yaw是否在范围内 */
inline bool YawInRange(Yaw_type yaw, double lf, double rg) {
  if (rg < lf) std::swap(lf, rg);
  return yaw > lf && yaw < rg;
}

#endif