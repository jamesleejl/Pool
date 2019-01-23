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

  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH);
  cue_ball = Vector2d(0.15, DIAMOND_LENGTH * 4);
  for (unsigned char i = 0; i < 7; ++i)
  {
    object_balls.push_back(Vector2d(DIAMOND_LENGTH * 0.2 + DIAMOND_LENGTH * 0.2 * i, DIAMOND_LENGTH * 4));
    opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 1.6 + DIAMOND_LENGTH * 0.2 * i, DIAMOND_LENGTH * 4.15));
  }

  initialize_pockets();
  initialize_table_edges();
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_info_table_obstructions();
  populate_shot_info_table_difficulty();
  populate_shot_path_table();

  auto end = std::chrono::system_clock::now();

  // for (unsigned char o = 0; o < EIGHT_BALL_INDEX + 1; ++o)
  // {
  //   for (unsigned char p = 0; p < pockets.size(); ++p)
  //   {
  //     for (unsigned char w = 0; w < WIDTH; ++w)
  //     {
  //       for (unsigned char l = 0; l < LENGTH; ++l)
  //       {
  //         if (w == 0 && l == DIAMOND_LENGTH * 4)
  //         {
  //           cout << shot_info_table[w][l][o][p] << endl;
  //         }
  //       }
  //     }
  //   }
  // }
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << endl << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
