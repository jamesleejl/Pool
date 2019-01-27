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

  ball_in_hand = false;
  eight_ball = get_vector_from_ball_position_in_diamonds(1, 6);
  cue_ball = get_vector_from_ball_position_in_diamonds(3.5, 5);
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(1, 1));
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(2, 1));
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(3, 1));
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(2, 2));
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(2, 3));
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(0, 3));
  object_balls.push_back(get_vector_from_ball_position_in_diamonds(4, 3));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));
  opponent_object_balls.push_back(get_vector_from_ball_position_in_diamonds(2.5, 0));

  initialize_pockets();
  initialize_table_edges();
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_info_table_obstructions();
  populate_shot_info_table_difficulty();
  populate_shot_path_table();
  populate_shot_info_and_path_table();
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
