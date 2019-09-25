#include "opencvdraw.h"
static int path_cnt = 0;

/* 操作：
 *  s  : 开始
 *  r  ： 重新开始
 * Esc ： 退出
 *  */

/* opencv draw Astar result */
void DrawWholeMap(int num, int row, int col, const Point2i_type& start,
                  const Point2i_type& goal,
                  const std::vector<Obstacle>& obstacle,
                  const std::list<CellPosState>& path) {
  cv::Mat img;
  std::string title = "map - " + std::to_string(num);
  cv::namedWindow(title, CV_WINDOW_NORMAL);

  /* Path smooth bezier curve */
  std::vector<DrawPathPos> final_path;
  PathSmooth(col, path, final_path);

  while (1) {
    DrawGridMap(img, row, col, start, goal, obstacle);

    DrawCar(img, final_path.front().pt, final_path.front().yaw);

    cv::imshow(title, img);

    while (cv::waitKey(0) != 's') {
      /* 's' -> start */
    }

    for (int i = 0; i < final_path.size(); ++i) {
      std::cout << "time: " << i << "  ";
      DrawGridMap(img, row, col, start, goal, obstacle);

      DrawCar(img, final_path[i].pt, final_path[i].yaw);

      cv::imshow(title, img);

      /* Esc 退出 */
      if (cv::waitKey(350) == 27) break;
    }

    bool recure_flg = false;
    while (1) {
      if (cv::waitKey(0) == 'r') {
        recure_flg = true;
        path_cnt = 0;
        break;
      } else
        break;
    }

    if (!recure_flg) break;
  }

  // /* 画网格线 */
  // DrawGridMap(img, row, col, start, goal, obstacle);

  // /* 画路径点 */
  // DrawPath(img, final_path);
  // cv::imshow(title, img);
  // cv::waitKey(0);
  cv::destroyWindow(title);
  path_cnt = 0;
}

/* opencv draw grid map */
void DrawGridMap(cv::Mat& img, int row, int col, const Point2i_type& start,
                 const Point2i_type& goal,
                 const std::vector<Obstacle>& obstacle) {
  //网格线宽
  const int line_thickness = OpencvdrawSize::grid_line_thinckness();
  const int edge_length = OpencvdrawSize::edge_length();

  img = cv::Mat(row * edge_length + line_thickness,
                col * edge_length + line_thickness, CV_8UC3,
                cv::Scalar::all(255));
  // grid line
  for (int16_t i = 0; i <= col; ++i) {
    cv::line(img, cv::Point(i * edge_length, 0),
             cv::Point(i * edge_length, row * edge_length), cv::Scalar::all(0),
             line_thickness);
  }
  for (int16_t i = 0; i <= row; ++i) {
    cv::line(img, cv::Point(0, i * edge_length),
             cv::Point(col * edge_length, i * edge_length), cv::Scalar::all(0),
             line_thickness);
  }

  // star -> green
  cv::circle(
      img,
      cv::Point(static_cast<int>(start.y * edge_length) + edge_length / 2,
                static_cast<int>(start.x * edge_length) + edge_length / 2),
      8, OpencvColor::Start(), cv::FILLED, cv::LINE_AA);

  // goal -> red
  cv::circle(
      img,
      cv::Point(static_cast<int>(goal.y * edge_length) + edge_length / 2,
                static_cast<int>(goal.x * edge_length) + edge_length / 2),
      8, OpencvColor::Goal(), cv::FILLED, cv::LINE_AA);

  // obstacle -> black
  int obs_cnt = 0;
  for (auto& elem : obstacle) {
    DrawObstacle(img, ++obs_cnt, path_cnt, elem);
  }
  ++path_cnt;
}

/* opencv draw car
 * input: central point of the car with opencv map size
 * */
void DrawCar(cv::Mat& img, const Point2d_type& central_point, Yaw_type yaw) {
  const int line_thickness = OpencvdrawSize::car_line_thinckness();

  std::vector<Point2d_type> border =
      GetBorder(central_point, yaw, CarSize::CarLength() - 2 * line_thickness,
                CarSize::CarWidth() - 2 * line_thickness);

  for (int i = 0; i < border.size(); ++i) {
    cv::line(img, SwapPointXY(border[i]),
             SwapPointXY(border[(i + 1) % border.size()]),
             cv::Scalar(255, 0, 0), line_thickness, cv::LINE_AA);
  }
  cv::circle(img, SwapPointXY(border[0]), 2, cv::Scalar(255, 0, 255), -1,
             cv::LINE_AA);
  std::cout << "car site: " << central_point << "  yaw: " << yaw << std::endl;
}

/* draw path */
void DrawPath(cv::Mat& img, const std::vector<DrawPathPos>& final_path) {
  for (auto& elem : final_path) {
    cv::circle(img, SwapPointXY(elem.pt), 1, cv::Scalar(255, 200, 0), -1,
               cv::LINE_AA);
    DrawCar(img, elem.pt, elem.yaw);
  }
}

/* opencv draw obstacle */
void DrawObstacle(cv::Mat& img, int order_num, int path_cnt,
                  const Obstacle& obs) {
  if (path_cnt < obs.path_set.size()) {
    Point2d_type curr_central = obs.path_set[path_cnt];
    const int line_thickness = OpencvdrawSize::obs_line_thinckness();
    const ObsSize_type size = ObsSize::getsize(obs.size_class);

    /* 获得障碍物边界点 */
    std::vector<Point2d_type> border =
        GetBorder(curr_central, obs.yaw, size.width - 2 * line_thickness,
                  size.height - 2 * line_thickness);

    std::vector<Point2i_type> temp_border(border.size());
    for (int i = 0; i < temp_border.size(); ++i) {
      temp_border[i] = SwapPointXY(border[i]);
    }

    cv::fillConvexPoly(img, temp_border, OpencvColor::Obstacle(), cv::LINE_AA);

    /* 显示障碍物编号 */
    Point2i_type font_central = SwapPointXY(obs.path_set[path_cnt]);
    font_central.x -= 6 * OpencvColor::FontSize();
    font_central.y += 6 * OpencvColor::FontSize();
    cv::putText(img, std::to_string(order_num), font_central,
                OpencvColor::FontType(), OpencvColor::FontSize(),
                OpencvColor::FontColor(), 1, cv::LINE_AA);
  }
}
