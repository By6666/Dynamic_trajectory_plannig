#include "execute.h"

bool SearchOneMap(int map_num) {
  //获得map信息
  GrideInput map_info(map_num);
  map_info.GetOneGrid();
#if DISPLAY
  map_info.PrintMap();  //打印原始map
#endif

  std::vector<Obstacle> dynamic_obs;  //动态障碍物信息
  /* obstacle info init */
  GenerateObstacle(map_info.get_obstacle_pos(), map_info.get_grid_rows(),
                   map_info.get_grid_columns(), map_info.get_start_yaw(),
                   dynamic_obs);

  int time_begin = 0;
  bool find_goal_flg = false;
  std::cout << "/*******************************************/" << std::endl;
  std::cout << "/**********file——" << map_num << "**********/" << std::endl;
  
  //数据传入，构造类对象
  Astar Astar_algorithm(map_info.get_grid_rows(), map_info.get_grid_columns(),
                        map_info.get_start_pos(), map_info.get_goal_pos(),
                        map_info.get_start_yaw(), dynamic_obs, time_begin);
  while (1) {
#if DISPLAY
    std::cout << "search times : " << ++time_begin << std::endl;
#endif
    if (Astar_algorithm.Execute()) {
#if DISPLAY
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      std::cout << "|final result: get goal successflly!!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
#endif
      find_goal_flg = true;
      break;
    } else {
#if DISPLAY
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
#endif
      ++Astar_algorithm.set_time_begin();
    }
  }
#if DISPLAY
  /* result path */
  Astar_algorithm.PrintResult();

  /* opencv draw */
  if (find_goal_flg)
    DrawWholeMap(map_num, Astar_algorithm.get_row(), Astar_algorithm.get_col(),
                 Astar_algorithm.get_start(), Astar_algorithm.get_goal(),
                 Astar_algorithm.get_obs_info(),
                 Astar_algorithm.get_final_path());
#endif

  return find_goal_flg;
}