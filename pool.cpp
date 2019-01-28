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
  auto start = std::chrono::system_clock::now();

  ball_in_hand = true;
  eight_ball = get_vector_from_ball_position_in_diamonds(3, 1);
  cue_ball = get_vector_from_ball_position_in_diamonds(0, 5);
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(1.25, 0.5));

  initialize_pockets();
  initialize_table_edges();
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  cout << "Populating shot info obstructions." << endl;
  populate_shot_info_table_obstructions();
  cout << "Populating shot info difficulty." << endl;
  populate_shot_info_table_difficulty();
  cout << "Populating shot path." << endl;
  populate_shot_path_table();
  cout << "Populating shot info and path." << endl;
  populate_shot_info_and_path_table();
  cout << "Populating selected shot." << endl;
  populate_selected_shot_table();

  Vector2d initial_cue_ball_position = get_cue_ball_position_for_runout();
  display_solution(initial_cue_ball_position);
  write_to_file(get_json_for_solution(initial_cue_ball_position));

  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << endl
            << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
