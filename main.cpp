#include <iostream>
#include <vector>
#include <set>
#include <chrono>
#include <ctime>
#include "./pool.h"
#include "./structs.h"

#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

int main()
{
  srand( time( NULL ) );
  vector<Ball> balls;
  Solution solution;
  do {
    balls.clear();
    for (unsigned int i = 0; i < 16; ++i) {
      Ball new_ball(0, 0);
      bool valid_ball = true;
      do {
        valid_ball = true;
        new_ball = generate_random_ball();
        for (unsigned int j = 0; j < balls.size(); ++j) {
          if (!balls_far_enough_apart(new_ball, balls[j])) {
            valid_ball = false;
            break;
          }
        }
      } while (!valid_ball);
      balls.push_back(new_ball);
    }
    initialize(
      balls[0], /* Cue ball */
      balls[1], /* Eight ball */
      {balls[2], balls[3], balls[4], balls[5], balls[6], balls[7], balls[8]}, /* Player balls */ //
      {balls[9], balls[10], balls[11], balls[12], balls[13], balls[14], balls[15]} /* Opponent balls */
    );
    populate_tables();
    solution = get_solution();
  } while (!solution.get_has_solution());
  write_to_file(solution.get_solution());
}
