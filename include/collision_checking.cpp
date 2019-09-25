#include "collision_checking.h"

/* 旋转obj */
void RotateObj(std::vector<Point2d_type>& car,
               const Point2d_type& central_point, Yaw_type yaw) {
  for (auto& elem : car) {
    Point2d_type temp = elem;
    elem.x = temp.x * cos(yaw) - temp.y * sin(yaw) + central_point.x;
    elem.y = temp.x * sin(yaw) + temp.y * cos(yaw) + central_point.y;
  }
}

/* *
 * 获得obj体边框四点
 * D-----A
 * | ——> |  车头方向
 * C-----B
 * A,B,C,D  ——>  0,1,2,3
 * */
std::vector<Point2d_type> GetBorder(const Point2d_type& central, Yaw_type yaw,
                                    double frame_length, double frame_width) {
  std::vector<Point2d_type> objposet(4);
  objposet.reserve(4);

  objposet[0].x = frame_length / 2.0;
  objposet[0].y = frame_width / 2.0;

  objposet[1].x = frame_length / 2.0;
  objposet[1].y = -frame_width / 2.0;

  objposet[2].x = -frame_length / 2.0;
  objposet[2].y = -frame_width / 2.0;

  objposet[3].x = -frame_length / 2.0;
  objposet[3].y = frame_width / 2.0;

  RotateObj(objposet, central, yaw);

  return objposet;
}

/* collision checking */
bool CollisionChecking(int col, int time, const Point2i_type& first_pos,
                       const Point2i_type& second_pos, Yaw_type yaw,
                       const std::vector<Obstacle>& obstacle_list) {
  /* 获得car的 CollisionObj*/
  CollisionObj car = CreareCollisionObj(
      GridmapToOpencvdraw(first_pos), GridmapToOpencvdraw(second_pos),
      CarSize::CarLength(), CarSize::CarWidth(), yaw);
  for (const auto& elem : obstacle_list) {
    int cur = time, next = time;
    if (time > (elem.path_set.size() - 1)) {
      continue;
    } else if (time < (elem.path_set.size() - 1)) {
      ++next;
    }
    CollisionObj temp_obs =
        CreareCollisionObj(elem.path_set[cur], elem.path_set[next],
                           ObsSize::getsize(elem.size_class).width,
                           ObsSize::getsize(elem.size_class).height, yaw);

    if (TwoObjCollisionChecking(car, temp_obs)) {
      return true;
    }
  }

  return false;
}

/* create two obj for collision checking */
CollisionObj CreareCollisionObj(Point2d_type start, Point2d_type end,
                                double obj_length, double obj_width,
                                Yaw_type yaw) {
  Point2d_type central((start.x + end.x) / 2.0, (start.y + end.y) / 2.0);
  double frame_length =
      sqrt(pow(start.x - end.x, 2.0) + pow(start.y - end.y, 2.0)) + obj_length;
  double frame_width = obj_width;

  /* 获得frame */
  std::vector<Point2d_type> frame =
      GetBorder(central, yaw, frame_length, frame_width);

  return CollisionObj(central, frame, frame_length / 2.0, frame_width / 2.0);
}

/* 两个obj 碰撞检测 */
bool TwoObjCollisionChecking(const CollisionObj& obj1,
                             const CollisionObj& obj2) {
  /* distence of two obj central */
  double central_dis = TwoPosDis(obj1.central, obj2.central);

  /* primary checking */
  if ((obj1.max_dis + obj2.max_dis) < central_dis) return false;
  if ((obj1.min_dis + obj2.min_dis) > central_dis) return true;

  /* collision checking using OBB */
  OBB obb(obj1.frame, obj2.frame);
  return obb.IsCollision();
}