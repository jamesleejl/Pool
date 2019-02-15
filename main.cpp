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
  initialize(
    Ball(4, 3), /* Cue ball */
    Ball(1, 4), /* Eight ball */
    {Ball(2, 1), Ball(1, 1), Ball(4, 2), Ball(3, 1), Ball(4, 1), Ball(5, 1), Ball(6, 1)}, /* Player balls */
    {Ball(1, 0), Ball(1, 0), Ball(1, 0), Ball(1, 0), Ball(1, 0), Ball(1, 0), Ball(1, 0)} /* Opponent balls */
  );
  populate_tables();
  write_to_file(get_json_for_solution());
}
