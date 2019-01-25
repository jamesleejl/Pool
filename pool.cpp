#include <iostream>
#include <vector>
#include <set>
#include <chrono>
#include <ctime>
#include "./pool.h"

#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

int main()
{
  cout << endl;
  auto start = std::chrono::system_clock::now();

  eight_ball = Vector2d(UNITS_PER_DIAMOND, 12);
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND, UNITS_PER_DIAMOND));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2, UNITS_PER_DIAMOND * 2));

  initialize_pockets();
  initialize_table_edges();
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_info_table_obstructions();
  populate_shot_info_table_difficulty();
  populate_shot_path_table();
  populate_selected_shot_table();

  cout << selected_shot_table[0][3][3].possible << endl;
  for (auto x : selected_shot_table[0][3][3].path_segments) {
    cout << x << endl;
  }
  cout << (int) selected_shot_table[0][3][3].next_x << endl;
  cout << (int) selected_shot_table[0][3][3].next_y << endl;
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << endl
            << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
