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

  bool get_ball_in_hand_solution = false;
  eight_ball = Vector2d(UNITS_PER_DIAMOND, 12);
  unsigned_char_coordinates_struct cue_ball;
  cue_ball.x = 7;
  cue_ball.y = 10;
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND, UNITS_PER_DIAMOND));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2, UNITS_PER_DIAMOND));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 3, UNITS_PER_DIAMOND));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2, UNITS_PER_DIAMOND * 2));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2, UNITS_PER_DIAMOND * 3));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 0, UNITS_PER_DIAMOND * 3));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 4, UNITS_PER_DIAMOND * 3));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2.5, 0));

  initialize_pockets();
  initialize_table_edges();
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_info_table_obstructions();
  populate_shot_info_table_difficulty();
  cout << shot_info_table[6][6][0][0].shot_obstructions << endl;
  populate_shot_path_table();
  populate_selected_shot_table();

  if (get_ball_in_hand_solution) {
    ball_in_hand_solution_struct ball_in_hand_solution = find_ball_in_hand_solution();
    if (!ball_in_hand_solution.possible) {
      cout << "No solution" << endl;
    } else {
      display_solution(true, ball_in_hand_solution.coordinates);
    }
  } else {
    display_solution(false, cue_ball);
  }
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << endl
            << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
