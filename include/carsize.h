#ifndef PLANNER_CARSIZE_H
#define PLANNER_CARSIZE_H

#include "typedef.h"

/* 车辆尺寸 */
namespace CarSize {
inline double kConstraintValue() { return M_PI / 6.0; }  //车辆转向角约束值
inline double CarLength() { return 46.0; }               //车长度，单位cm
inline double CarWidth() { return 18.0; }                //车宽度，单位cm
}  // namespace CarSize

/* opencv draw size */
namespace OpencvdrawSize {
inline int edge_length() { return 30; }
inline int car_line_thinckness() { return 2; }
inline int grid_line_thinckness() { return 1; }
inline int obs_line_thinckness() { return car_line_thinckness(); }
}  // namespace OpencvdrawSize

/* opencv color */
namespace OpencvColor {
inline cv::Scalar Obstacle() { return cv::Scalar(33, 36, 41); }
inline cv::Scalar Start() { return cv::Scalar(0, 255, 0); }
inline cv::Scalar Goal() { return cv::Scalar(0, 0, 255); }
inline cv::Scalar FontColor() { return cv::Scalar(0, 215, 255); }
inline int FontType() { return CV_FONT_HERSHEY_COMPLEX_SMALL; }
inline double FontSize() { return 0.75; }
}  // namespace OpencvColor

#endif