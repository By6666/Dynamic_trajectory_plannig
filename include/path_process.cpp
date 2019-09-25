#include "path_process.h"

/* car path smooth */
bool CarPathSmooth(int col, const std::list<CellPosState>& path,
                   std::vector<DrawPathPos>& final_path) {
  if (path.size() < 2) {
    return false;
  }
  /* change pt size from gridmap to opencvdraw */
  std::vector<DrawPathPos> temp_stg;
  temp_stg.reserve(path.size());
  for (auto& elem : path) {
    temp_stg.push_back(
        DrawPathPos(GridmapToOpencvdraw(DecodeID(elem.id, col)), elem.yaw));
  }

  // final_path = temp_stg;
  // return true;

  /* path smooth */
  PathSmooth(temp_stg, final_path);

  /* 打印路径 */
  // std::cout << "all path size : " << final_path.size() << std::endl;
  // int cnt = 0;
  // for (auto& elem : final_path) {
  //   std::cout << ++cnt << "   " << elem.pt << "  " << elem.yaw << std::endl;
  // }

  return true;
}

/* obstacle path smooth */
void ObsPathSmooth(std::vector<Obstacle>& obstacle) {
  for (auto& elem : obstacle) {
    std::vector<DrawPathPos> temp_stg =
        ObstacleToDrawPathPos(elem.path_set, elem.yaw);
    std::vector<DrawPathPos> final_path;
    PathSmooth(temp_stg, final_path);
    elem.path_set.clear();
    for (auto& iter : final_path) {
      elem.path_set.push_back(iter.pt);
    }
  }
}

/* obs pos type to opencv draw type */
std::vector<DrawPathPos> ObstacleToDrawPathPos(
    const std::vector<Point2d_type>& obs_path, Yaw_type obs_yaw) {
  std::vector<DrawPathPos> temp;
  temp.reserve(obs_path.size());
  for (auto& elem : obs_path) {
    temp.push_back(DrawPathPos(elem, obs_yaw));
  }
  return temp;
}

/* path smooth */
bool PathSmooth(const std::vector<DrawPathPos>& prime_path,
                std::vector<DrawPathPos>& final_path) {
  /* 路径分段 */
  int last_divide_pos = 0;
  bool capture_flg = false;
  for (int i = 0; i < prime_path.size() - 1; i++) {
    /* 获得分段点 */
    if (prime_path[i].yaw != prime_path[i + 1].yaw) {
      if (!capture_flg) {
        PathInterpoletion(last_divide_pos, i - 1, PATHDIVID, prime_path,
                          final_path);
        last_divide_pos = i - 1;

        capture_flg = true;
      } else {
        std::vector<Point2d_type> posset;
        posset.push_back(prime_path[last_divide_pos].pt);
        posset.push_back(prime_path[last_divide_pos + 1].pt);
        posset.push_back(prime_path[i].pt);
        posset.push_back(prime_path[i + 1].pt);

        ThridOrderBezier(PATHDIVID * (i - last_divide_pos + 1), posset,
                         final_path);
        ++i;
        last_divide_pos = i;
        capture_flg = false;
      }
    }
  }
  PathInterpoletion(last_divide_pos, prime_path.size() - 1, PATHDIVID,
                    prime_path, final_path);
  final_path.push_back(prime_path.back());

  return true;
}

/* 直线路段 插值 */
void PathInterpoletion(int start, int end, int pre_part_num,
                       const std::vector<DrawPathPos>& prime_path,
                       std::vector<DrawPathPos>& final_path) {
  for (int i = start; i < end; ++i)
    PrePartInterpoletion(prime_path[i], prime_path[i + 1], final_path,
                         pre_part_num);
}

/* 分段进行插值 */
void PrePartInterpoletion(const DrawPathPos& start, const DrawPathPos& end,
                          std::vector<DrawPathPos>& final_path,
                          int pre_part_num) {
  double x_step = (end.pt.x - start.pt.x) / pre_part_num;
  double y_step = (end.pt.y - start.pt.y) / pre_part_num;
  DrawPathPos temp(start);
  for (int i = 0; i < pre_part_num; ++i) {
    temp.pt.x = start.pt.x + x_step * i;
    temp.pt.y = start.pt.y + y_step * i;

    final_path.push_back(temp);
  }
}