#ifndef PLANNER_OPENCVDRAW_H
#define PLANNER_OPENCVDRAW_H

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "carsize.h"
#include "collision_checking.h"
#include "coordinate_change.h"
#include "obs_create.h"
#include "path_process.h"
#include "state.h"
#include "typedef.h"

/* opencv draw Astar result */
void DrawWholeMap(int num, int row, int col, const Point2i_type& start,
                  const Point2i_type& goal,
                  const std::vector<Obstacle>& obstacle,
                  const std::list<CellPosState>& path);

/* opencv draw grid map */
void DrawGridMap(cv::Mat& img, int row, int col, const Point2i_type& start,
                 const Point2i_type& goal,
                 const std::vector<Obstacle>& obstacle);

/* opencv draw car */
void DrawCar(cv::Mat& img, const cv::Point2d& central_point, Yaw_type yaw);

/* opencv draw obstacle */
void DrawObstacle(cv::Mat& img, int order_num, int path_cnt,
                  const Obstacle& obs);

/* draw path */
void DrawPath(cv::Mat& img, const std::vector<DrawPathPos>& final_path);

#endif