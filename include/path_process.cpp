#include "path_process.h"

/* path smooth */
bool PathSmooth(int col, const std::list<CellPosState>& path,
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
  final_path = temp_stg;
  return true;

  /* 路径分段 */
  int last_divide_pos = 0;
  bool capture_flg = false;
  for (int i = 0; i < temp_stg.size() - 1; i++) {
    /* 获得分段点 */
    if (temp_stg[i].yaw != temp_stg[i + 1].yaw) {
      if (!capture_flg) {
        PathInterpoletion(temp_stg[last_divide_pos], temp_stg[i - 1],
                          i - last_divide_pos, PATHDIVID, final_path);
        last_divide_pos = i - 1;

        capture_flg = true;
      } else {
        std::vector<Point2d_type> posset;
        posset.push_back(temp_stg[last_divide_pos].pt);
        posset.push_back(temp_stg[last_divide_pos + 1].pt);
        posset.push_back(temp_stg[i].pt);
        posset.push_back(temp_stg[i + 1].pt);

        ThridOrderBezier(4 * PATHDIVID * (i - last_divide_pos - 1), posset,
                         final_path);
        ++i;
        last_divide_pos = i;
        capture_flg = false;
      }
    }
  }
  final_path.push_back(temp_stg.back());

  /* 打印路径 */
  std::cout << "all path size : " << final_path.size() << std::endl;
  int cnt = 0;
  for (auto& elem : final_path) {
    std::cout << ++cnt << "   " << elem.pt << "  " << elem.yaw << std::endl;
  }

  return true;
}

/* 直线路段 插值 */
void PathInterpoletion(const DrawPathPos& start, const DrawPathPos& end,
                       int size, int pre_part_num,
                       std::vector<DrawPathPos>& final_path) {
  if (start.pt == end.pt) return;
  /* 总段数 */
  pre_part_num = pre_part_num * (size - 1);

  double x_step = (end.pt.x - start.pt.x) / pre_part_num;
  double y_step = (end.pt.y - start.pt.y) / pre_part_num;

  DrawPathPos temp(start);
  for (int i = 0; i < pre_part_num; ++i) {
    temp.pt.x = start.pt.x + x_step * i;
    temp.pt.y = start.pt.y + y_step * i;

    final_path.push_back(temp);
  }
}