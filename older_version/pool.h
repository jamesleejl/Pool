#include <iomanip>
#include <sstream>
#include <iostream>
#include <vector>
#include <set>
#include <chrono>
#include <ctime>
#include <string>
#include <fstream>
#include "./json.hpp"
#include "Eigen/Dense"
using namespace Eigen;
using namespace std;
using json = nlohmann::json;

// TODO: Extend this to python: https://docs.python.org/3/extending/building.html
// TODO: Handle bank shots.
// TODO: Throttle raspberry pi? https://www.howtoforge.com/how-to-limit-cpu-usage-of-a-process-with-cpulimit-debian-ubuntu

bool ball_in_hand = false;


const Vector2d find_ball_in_hand_solution()
{
  short max_object_ball_index = (short)pow(2, object_balls.size()) - 1;
  double x = 0;
  double y = 0;
  double minimum_difficulty = std::numeric_limits<double>::infinity();
  for (short w = 0; w <= WIDTH; ++w)
  {
    for (short l = 0; l <= LENGTH; ++l)
    {
      if (selected_shot_table[w][l][max_object_ball_index].total_weighted_difficulty < minimum_difficulty)
      {
        x = w;
        y = l;
        minimum_difficulty = selected_shot_table[w][l][max_object_ball_index].total_weighted_difficulty;
      }
    }
  }
  return move_ball_in_from_rails(Vector2d(x, y));
}

const Vector2d get_cue_ball_position_for_runout()
{
  if (ball_in_hand) {
    return find_ball_in_hand_solution();
  }
  return cue_ball;
}

string zero_based_index_to_one_based_index_string(short index)
{
  return to_string(index + 1);
}

string object_ball_index_to_string(short index)
{
  Vector2d object_ball;
  string ret = "";
  if (index == object_balls.size())
  {
    ret = "Eight ball";
    object_ball = eight_ball;
  }
  else
  {
    ret = zero_based_index_to_one_based_index_string(index);
    object_ball = object_balls[index];
  }
  ret += " " + vector_to_string(object_ball);
  return ret;
}

string pocket_to_string(short pocket_index)
{
  switch (pocket_index)
  {
  case 0:
    return "bottom left";
  case 1:
    return "bottom right";
  case 2:
    return "middle left";
  case 3:
    return "middle right";
  case 4:
    return "top left";
  case 5:
    return "top right";
  }
  return "";
}

// TODO: This hacks everything to a spin of stun.
string spin_to_string(short spin_index)
{
  return "stun";
  switch (spin_index)
  {
  case 0:
    return "heavy draw";
  case 1:
    return "light draw";
  case 2:
    return "stun";
  case 3:
    return "light follow";
  case 4:
    return "heavy follow";
  }
  return "";
}

string strength_to_string(short strength)
{
  return to_string(strength + 1) +
         " - " +
         to_string((int)round(strength_to_distance(strength) / (MAX_DIAMONDS * UNITS_PER_DIAMOND) * 100)) + "%";
}

string unsigned_short_int_coordinates_struct_to_string(unsigned_short_int_coordinates_struct coordinates)
{
  return vector_to_string(Vector2d(coordinates.x, coordinates.y));
}

/*
std::ostream &operator<<(std::ostream &o, const shot_info_struct &shot_info)
{
  o << shot_info.difficulty << endl;
  o << shot_info.shot_obstructions << endl;
  return o;
}

// Returns true if the point p interescts the given segment specified by the start and end points.
// TODO: Might never be used.
//
bool point_intersects_segment(Vector2d p, Vector2d segment_start, Vector2d segment_end)
{
  Vector2d start_to_point = p - segment_start;
  Vector2d end_to_point = p - segment_end;
  Vector2d start_to_end = segment_end - segment_start;
  return start_to_point.dot(start_to_end) > 0 && end_to_point.dot(start_to_end) < 0;
}

*/
std::ostream &operator<<(std::ostream &o, const obstructions_struct &obstructions)
{
  o << "Permanent obstruction: " << obstructions.has_permanent_obstruction << endl;
  o << "Object ball obstructions: " << endl;
  for (short index : obstructions.obstructing_object_balls)
  {
    o << (int)index << " ";
  }
  return o;
}

// TODO: This converts to diamonds automatically.
string vector_to_string(Vector2d vec)
{
  stringstream stream;
  stream << "(";
  stream << fixed << setprecision(1) << vec.x() / UNITS_PER_DIAMOND;
  stream << ", ";
  stream << fixed << setprecision(1) << vec.y() / UNITS_PER_DIAMOND;
  stream << ")";
  return stream.str();
}

string path_to_string(vector<Vector2d> path)
{
  string ret = "";
  for (auto vec : path)
  {
    string vec_string = vector_to_string(vec);
    ret += vec_string + " ";
  }
  return ret;
}

/**
 * Balls cannot be within BALL_RADIUS distance of the rail. Because the ball is not a point,
 * we must push them away from the rail by the ball's radius.
 */
Vector2d get_vector_from_ball_position_in_diamonds(double x_position_in_diamonds, double y_position_in_diamonds) {
  double x_position_in_units = x_position_in_diamonds * UNITS_PER_DIAMOND;
  double y_position_in_units = y_position_in_diamonds * UNITS_PER_DIAMOND;
  return move_ball_in_from_rails(Vector2d(x_position_in_units, y_position_in_units));
}

