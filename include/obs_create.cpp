#include "obs_create.h"

/* 生成obs对象集合 */
void GenerateObstacle(std::vector<ID_SIZE>& obs, int row, int col, Yaw_type yaw,
                      std::vector<Obstacle>& dynamic_obs) {
  if (!(yaw == 0.0 || yaw == M_PI_2)) std::reverse(obs.begin(), obs.end());
  std::set<ID_SIZE> closelist;
  for (auto& elem : obs) {
    if (!IsInSet(closelist, elem)) {
      closelist.insert(elem);  //加入closelist防止再次被search
      ID_SIZE temp = elem;
      int i = 0;
      while (1) {
        temp = GetNextID(DecodeID(temp, col), yaw, col);
        if (IsInVector(obs, temp) && !IsInSet(closelist, temp)) {
          ++i;
          closelist.insert(temp);
          continue;
        } else {
          break;
        }
      }
      dynamic_obs.push_back(
          Obstacle(i + 1, GetObsCentralPos(elem, i + 1, col, yaw), yaw));
    }
  }

//   std::cout << "obstacle change   size : " << dynamic_obs.size() << std::endl;
//   for (auto& elem : dynamic_obs) {
//     std::cout << "pos : " << elem.prime_pos << "  size :" << elem.size_class
//               << " | " << ObsSize::getsize(elem.size_class) << std::endl;
//   }

  /* 障碍物轨迹预测 */
  ObstaclePathPrediction(dynamic_obs, row, col);
}

/* 计算障碍物中心点坐标 */
Point2d_type GetObsCentralPos(ID_SIZE id, int size_class, int col,
                              Yaw_type yaw) {
  Point2d_type prime = GridmapToOpencvdraw(DecodeID(id, col));

  Point2d_type temp((size_class - 1) * OpencvdrawSize::edge_length() / 2.0, 0);
  RotateCoordinate(temp, yaw, prime);

  return prime;
}

/* 获得当前障碍物下一个点的id */
ID_SIZE GetNextID(const Point2i_type& pt, Yaw_type yaw, int col) {
  Point2i_type temp(1, 0);
  return CodeID(RotateCoordinate(temp, yaw, pt), col);
}

/* 障碍物路径预测 */
void ObstaclePathPrediction(std::vector<Obstacle>& obstacle, int row, int col) {
  for (auto& elem : obstacle) {
    int cnt = 0;
    while (cnt < ObsPathNumUpper) {
      elem.MoveStep(row, col);
      if (!IsInGridmap(row, col, elem.last_pos)) break;
      ++cnt;
    }
  }
}