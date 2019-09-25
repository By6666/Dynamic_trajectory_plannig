#include "bezier_curve.h"

// 三阶bezier curve
void ThridOrderBezier(int pos_num, const std::vector<Point2d_type>& posset,
                      std::vector<DrawPathPos>& path) {
  double t = 0.0;
  double step_t = 1.0 / pos_num;
  for (; t < 1 - step_t; t += step_t) {
    DrawPathPos temp;
    temp.pt.x = ThridOrderBezierFormula(posset[0].x, posset[1].x, posset[2].x,
                                        posset[3].x, t);
    temp.pt.y = ThridOrderBezierFormula(posset[0].y, posset[1].y, posset[2].y,
                                        posset[3].y, t);

    temp.yaw = atan2(ThridOrderBezierDifferential(posset[0].y, posset[1].y,
                                                  posset[2].y, posset[3].y, t),
                     ThridOrderBezierDifferential(posset[0].x, posset[1].x,
                                                  posset[2].x, posset[3].x, t));
    path.push_back(temp);
  }
}