#include <stdint.h>

#include <time.h>
#include <iostream>

#include "include/execute.h"
#include "include/grid_input.h"

int main() {
  // int success_cnt = 0;
  // std::vector<int16_t> stg;
  // clock_t start, end;
  // start = clock();
  // for (int16_t i = 0; i < 960; ++i) {
  //   // DrawOneMap(i + 1);
  //   if (SearchOneMap(i + 1))
  //     ++success_cnt;
  //   else
  //     stg.push_back(i + 1);
  // }
  // end = clock();

  // std::cout << "success num : " << success_cnt << std::endl;
  // std::cout << "fail num : " << stg.size() << std::endl;
  // for (auto& elem : stg) {
  //   std::cout << elem << " ";
  // }
  // std::cout << std::endl;

  // /* 输出运行的时间 */
  // printf("Spend time %.5f seconds!!\n", (float)(end - start) /
  // CLOCKS_PER_SEC); std::cout << std::endl << std::endl;

  while (1) {
    int num = 0;
    std::cout << "Please input map_num: ";
    std::cin >> num;
    SearchOneMap(num);
  }

  return 0;
}