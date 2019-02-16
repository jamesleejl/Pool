#include "./pool.h"
#include "./structs.h"
#include "./constants.h"
#include <iomanip>
#include <sstream>
#include <iostream>
#include <set>
#include <chrono>
#include <ctime>
#include <string>
#include <fstream>
#include <limits>
#include "./json.hpp"
#include <math.h>
#include <random>

using namespace std;
using json = nlohmann::json;

vector<Pocket> pockets;
Vector2d cue_ball;
Vector2d eight_ball;
vector<Vector2d> player_balls;
vector<Vector2d> player_balls_without_eight_ball;
vector<Vector2d> opponent_balls;
vector<Vector2d> bottom_edge;
vector<Vector2d> right_edge;
vector<Vector2d> top_edge;
vector<Vector2d> left_edge;
vector<vector<BallObstructions>> player_ball_to_pocket_obstructions_table;
vector<vector<GhostBall>> ghost_ball_position_table;
vector<vector<vector<vector<Shot>>>> shot_table;
vector<vector<vector<vector<vector<vector<PostShot>>>>>> post_shot_table;
vector<vector<vector<SelectedShot>>> selected_shot_table;

bool balls_far_enough_apart(const Ball& ball1, const Ball& ball2) {
  return (ball2.to_vec2d() - ball1.to_vec2d()).norm() > UNITS_PER_DIAMOND / 2;
}

Ball generate_random_ball() {
  double random_width = ((double)rand() / (double)(RAND_MAX)) * 4;
  double random_length = ((double)rand() / (double)(RAND_MAX)) * 8;
  return Ball(random_length, random_width);
}

Vector2d reflect_point_in_x_axis(const Vector2d& point) {
  return Vector2d(point.x(), -1 * point.y());
}

Vector2d reflect_point_in_y_axis(const Vector2d& point) {
  return Vector2d(-1 * point.x(), point.y());
}

double rail_path_scaling_factor_given_absolute_angle_to_edge(double absolute_angle_to_edge) {
  return 1.0 / (cos(absolute_angle_to_edge) + 1);
}

double absolute_angle_to_edge(double angle, Edge edge) {
  double absolute_angle = fabs(angle);
  if (edge == Edge::TOP || edge == Edge::BOTTOM) {
    return absolute_angle;
  }
  return M_PI / 2 - absolute_angle;
}

double slope_to_angle(double slope) {
  return atan(slope);
}

void write_to_file(string json)
{
  ofstream myfile;
  myfile.open(GAME_DATA_FILE);
  myfile << "game_data = " << json << "\n";
  myfile.close();
}

Coordinates get_ball_in_hand_solution() {
  short combo = (short)pow(2, player_balls_without_eight_ball.size()) - 1;
  double difficulty = std::numeric_limits<double>::infinity();
  Coordinates ret(-1, -1);
  for (int l = 0; l <= LENGTH; ++l) {
    for (int w = 0; w <= WIDTH; ++w) {
      if (selected_shot_table[l][w][combo].get_possible() && selected_shot_table[l][w][combo].get_total_weighted_difficulty() < difficulty) {
        difficulty = selected_shot_table[l][w][combo].get_total_weighted_difficulty();
        ret = Coordinates(l, w);
      }
    }
  }
  return ret;
}

// TODO: Test
string get_json_for_solution()
{
  json game_data;

  short combo = (short)pow(2, player_balls_without_eight_ball.size()) - 1;

  // Coordinates coords(round(cue_ball.x()), round(cue_ball.y())); TODO: Uncomment for non ball in hand.
  Coordinates coords = get_ball_in_hand_solution();

  short shot_number = 1;
  game_data["ball_in_hand"] = false;
  game_data["cue_ball"] = {to_diamonds(cue_ball.x()), to_diamonds(cue_ball.y())};
  game_data["eight_ball"] = {to_diamonds(eight_ball.x()), to_diamonds(eight_ball.y())};
  game_data["object_balls"] = {};
  game_data["turns"] = {};
  game_data["has_solution"] = false;
  game_data["object_balls"].push_back(game_data["eight_ball"]);
  for (unsigned short o = 0; o < player_balls_without_eight_ball.size(); ++o)
  {
    game_data["object_balls"].push_back({to_diamonds(player_balls_without_eight_ball[o].x()), to_diamonds(player_balls_without_eight_ball[o].y())});
  }
  game_data["opponent_object_balls"] = {};
  for (unsigned short o = 0; o < opponent_balls.size(); ++o)
  {
    game_data["opponent_object_balls"].push_back({to_diamonds(opponent_balls[o].x()), to_diamonds(opponent_balls[o].y())});
  }
  do
  {
    // TODO: Hacky
    short coords_x = coords.get_x();
    if (coords_x < 0) {
      coords_x = 0;
    } else if (coords_x > LENGTH) {
      coords_x = LENGTH;
    }
    short coords_y = coords.get_y();
    if (coords_y < 0) {
      coords_y = 0;
    } else if (coords_y > WIDTH) {
      coords_y = WIDTH;
    }
    coords = Coordinates(coords_x, coords_y);
    SelectedShot selected_shot = selected_shot_table[coords.get_x()][coords.get_y()][combo];
    if (!selected_shot.get_possible())
    {
      return game_data.dump();
    }
    game_data["has_solution"] = true;
    json turn;
    turn["shot_number"] = shot_number;
    Vector2d fixed_cue_ball_position = move_ball_in_from_rails(Vector2d(coords.get_x(), coords.get_y()));
    turn["cue_ball"] = {to_diamonds(fixed_cue_ball_position.x()), to_diamonds(fixed_cue_ball_position.y())};
    turn["object_ball_index"] = selected_shot.get_object_ball();
    turn["pocket_index"] = selected_shot.get_pocket();
    turn["pocket_coords"] = {to_diamonds(pockets[selected_shot.get_pocket()].get_position().x()), to_diamonds(pockets[selected_shot.get_pocket()].get_position().y())};
    turn["runout_difficulty"] = selected_shot.get_total_weighted_difficulty();
    turn["shot_difficulty"] = selected_shot.get_current_weighted_difficulty();
    turn["strength"] = selected_shot.get_strength();
    turn["spin"] = selected_shot.get_spin();

    json path;
    for (unsigned short i = 0; i < selected_shot.get_cue_ball_path().size(); ++i)
    {
      path.push_back({to_diamonds(selected_shot.get_cue_ball_path()[i].x()), to_diamonds(selected_shot.get_cue_ball_path()[i].y())});
    }
    turn["path"] = path;
    game_data["turns"].push_back(turn);
    if (combo == 0)
    {
      break;
    }
    coords.set_x(selected_shot.get_cue_ball_final_position_x_coordinate());
    coords.set_y(selected_shot.get_cue_ball_final_position_y_coordinate());
    combo = selected_shot.get_next_combo();
    shot_number += 1;
  } while (true);
  return game_data.dump();
}

// TODO: Test.
void populate_eight_ball_in_selected_shot_table()
{
  for (unsigned short l = 0; l <= LENGTH; ++l)
  {
    for (unsigned short w = 0; w <= WIDTH; ++w)
    {
      SelectedShot &selected_shot = selected_shot_table[l][w][0];
      for (unsigned short p = 0; p < pockets.size(); ++p)
      {
        for (unsigned short st = 0; st < NUM_STRENGTHS; ++st)
        {
          for (unsigned short sp = 0; sp < Spin::MAX_NUM_SPINS; ++sp)
          {
            PostShot &post_shot = post_shot_table[l][w][eight_ball_index()][p][st][sp];
            if (!post_shot.get_possible())
            {
              continue;
            }
            if (post_shot.get_weighted_shot_difficulty() < selected_shot.get_total_weighted_difficulty())
            {
              selected_shot.set_possible(true);
              selected_shot.set_strength(st);
              selected_shot.set_spin(sp);
              selected_shot.set_object_ball(0);
              selected_shot.set_pocket(p);
              selected_shot.set_total_weighted_difficulty(post_shot.get_weighted_shot_difficulty());
              selected_shot.set_current_weighted_difficulty(post_shot.get_weighted_shot_difficulty());
              selected_shot.set_cue_ball_path(post_shot.get_cue_ball_path());
            }
          }
        }
      }
    }
  }
}

vector<Coordinates> get_coordinates_between_points(Vector2d start, Vector2d end) {
  // We derive the parameters for representing the line as
  // ax + by = c.
  double a = end.y() - start.y();
  double b = start.x() - end.x();
  double c = a * start.x() + b * start.y();
  short left_x = (short) ceil(min(start.x(), end.x()));
  short right_x = (short) floor(max(start.x(), end.x()));
  vector<Coordinates> ret;
  for (short x = left_x; x <= right_x; ++x) {
    ret.push_back(Coordinates(x, round((c - a * x) / b)));
  }
  return ret;
}

/**
 * This assumes that position is made 50% of the time, 25% it's shot too softly and 25% of the time it's shot too hard.
 * In the case of the misses, the strength unit is increased by one.
 * TODO: Finish and test
 */
void populate_single_combination_in_selected_shot_table(int combo, set<short> &balls)
{
  for (auto ball : balls)
  {
    int next_combo = combo - (1 << (ball - 1));
    for (unsigned short l = 0; l <= LENGTH; ++l)
    {
      for (unsigned short w = 0; w <= WIDTH; ++w)
      {
        SelectedShot &selected_shot = selected_shot_table[l][w][combo];
        for (unsigned short p = 0; p < pockets.size(); ++p)
        {
          for (unsigned short st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned short sp = 0; sp < Spin::MAX_NUM_SPINS; ++sp)
            {
              PostShot &post_shot = post_shot_table[l][w][ball][p][st][sp];
              if (!post_shot.get_possible()) {
                continue;
              }
              const set<short>& obstructing_player_balls = post_shot.get_obstructions().get_obstructing_player_balls();
              bool has_obstruction = false;
              for (auto ball : balls) {
                if (obstructing_player_balls.find(ball) != obstructing_player_balls.end()) {
                  has_obstruction = true;
                  break;
                }
              }
              if (has_obstruction) {
                continue;
              }
              Vector2d expected_cue_ball_position = post_shot.get_final_position();
              // Missed strengths are +/- 1.
              // TODO Missed shots are all equally likely.
              // TODO: Fix this algorithm for imperfect shots.
              vector<Coordinates> possible_cue_ball_locations;
              if (st >= 2) {
                PostShot &post_shot_short = post_shot_table[l][w][ball][p][st - 2][sp];
                if (!post_shot_short.get_possible()) {
                  continue;
                }
                vector<Coordinates> short_coordinates =
                  get_coordinates_between_points(expected_cue_ball_position, post_shot_short.get_final_position());
                possible_cue_ball_locations.insert(possible_cue_ball_locations.end(), short_coordinates.begin(), short_coordinates.end());
              }

              if (st != NUM_STRENGTHS - 1) {
                PostShot &post_shot_long = post_shot_table[l][w][ball][p][st + 1][sp];
                if (!post_shot_long.get_possible()) {
                  continue;
                }
                vector<Coordinates> long_coordinates =
                  get_coordinates_between_points(expected_cue_ball_position, post_shot_long.get_final_position());
                possible_cue_ball_locations.insert(possible_cue_ball_locations.end(), long_coordinates.begin(), long_coordinates.end());
              }
              double average_remaining_runout_weighted_difficulty = 0;
              for (Coordinates coordinates : possible_cue_ball_locations) {
                average_remaining_runout_weighted_difficulty += selected_shot_table[coordinates.get_x()][coordinates.get_y()][next_combo].get_total_weighted_difficulty();
              }
              Coordinates expected_cue_ball_coordinates = Coordinates((short) round(expected_cue_ball_position.x()), (short) round(expected_cue_ball_position.y()));
              average_remaining_runout_weighted_difficulty += possible_cue_ball_locations.size() * selected_shot_table[expected_cue_ball_coordinates.get_x()][expected_cue_ball_coordinates.get_y()][next_combo].get_total_weighted_difficulty();
              average_remaining_runout_weighted_difficulty /= 2 * possible_cue_ball_locations.size();
              double total_weighted_difficulty = post_shot.get_weighted_shot_difficulty() + average_remaining_runout_weighted_difficulty;
              if (total_weighted_difficulty < selected_shot.get_total_weighted_difficulty())
              {
                selected_shot.set_possible(true);
                selected_shot.set_strength(st);
                selected_shot.set_spin(sp);
                selected_shot.set_object_ball(ball);
                selected_shot.set_pocket(p);
                selected_shot.set_current_weighted_difficulty(post_shot.get_weighted_shot_difficulty());
                selected_shot.set_total_weighted_difficulty(total_weighted_difficulty);
                selected_shot.set_cue_ball_path(post_shot.get_cue_ball_path());
                selected_shot.set_cue_ball_final_position_x_coordinate(expected_cue_ball_coordinates.get_x());
                selected_shot.set_cue_ball_final_position_y_coordinate(expected_cue_ball_coordinates.get_y());
                selected_shot.set_next_combo(next_combo);
              }
            }
          }
        }
      }
    }
  }
}

set<short> get_set_from_combination(int n, int combo) {
  set<short> ret;
  for (int i = 0; i < n; ++i)
  {
    if ((combo >> i) & 1)
      ret.insert(i + 1);
  }
  return ret;
}
/**
 * Process a given combination of object balls.
 * combo represents the indices of object balls to consider. It is represented as a number.
 * TODO: test
 */
void process_object_ball_combination(int num_object_balls, int combo)
{
  cout << combo << endl;
  set<short> balls = get_set_from_combination(num_object_balls, combo);
  populate_single_combination_in_selected_shot_table(combo, balls);
}

vector<int> generate_all_combinations(short n, short k) {
  vector<int> ret;
  int combo = (1 << k) - 1; // k bit sets
  while (combo < 1 << n)
  {
    ret.push_back(combo);
    int x = combo & -combo;
    int y = combo + x;
    int z = (combo & ~y);
    combo = z / x;
    combo >>= 1;
    combo |= y;
  }
  return ret;
}

// TODO: Test this.
void process_object_ball_combinations(int num_object_balls, int num_elements)
{
  vector<int> object_ball_combinations =
    generate_all_combinations(num_object_balls, num_elements);
  for (unsigned int combo : object_ball_combinations) {
    process_object_ball_combination(num_object_balls, combo);
  }
}

/**
 * Populates the selected shot table.
 * TODO: Test
 */
void populate_selected_shot_table()
{
  selected_shot_table =
    vector<vector<vector<SelectedShot>>>(
      LENGTH + 1, vector<vector<SelectedShot>>(
        WIDTH + 1, vector<SelectedShot>((int)pow(2, player_balls_without_eight_ball.size()))));
  populate_eight_ball_in_selected_shot_table();
  for (unsigned short i = 1; i <= player_balls_without_eight_ball.size(); ++i)
  {
    process_object_ball_combinations(player_balls_without_eight_ball.size(), i);
  }
}

vector<Vector2d> get_path_with_reflections(vector<Vector2d> path) {
  if (path.size() <= 1) {
    return path;
  }
  vector<Vector2d> path_with_reflections;
  vector<Vector2d> empty_path;
  Vector2d start = path[0];
  path_with_reflections.push_back(start);
  for (unsigned int i = 1; i < path.size(); ++i) {
    Vector2d end = start + path[i];
    do {
      RailIntersection rail_intersection = get_rail_intersection(start, end);
      if (!rail_intersection.get_possible()) {
        return empty_path;
      }
      if (!rail_intersection.has_intersection()) {
        path_with_reflections.push_back(end);
        start = end;
        break;
      }
      start = rail_intersection.get_intersection_point();
      path_with_reflections.push_back(start);
      end = start + rail_intersection.get_end_vector();
      for (unsigned int j = i + 1; j < path.size(); ++j) {
        if (rail_intersection.get_intersection_edge() == Edge::TOP || rail_intersection.get_intersection_edge() == Edge::BOTTOM) {
          path[j] = reflect_point_in_x_axis(path[j]);
        } else {
          path[j] = reflect_point_in_y_axis(path[j]);
        }
        path[j] = path[j] * rail_intersection.get_scaling_factor();
      }
   } while (true);
  }
  return path_with_reflections;
}

Vector2d apply_reflection(Vector2d point, Edge edge) {
  switch (edge) {
    case LEFT:
      return Vector2d(left_edge[0].x() * 2 - point.x(), point.y());
    case RIGHT:
      return Vector2d(right_edge[0].x() * 2 - point.x(), point.y());
    case BOTTOM:
      return Vector2d(point.x(), bottom_edge[0].y() * 2 - point.y());
    case TOP:
      return Vector2d(point.x(), top_edge[0].y() * 2 - point.y());
  }
  return Vector2d(-1, -1);
}

// TODO: Test.
RailIntersection get_rail_intersection(Vector2d start, Vector2d end) {
  RailIntersection ret;
  // Should still work even if m is NaN.
  double m = (end.y() - start.y()) / (end.x() - start.x());
  double angle = slope_to_angle(m);
  // Left side
  double left_edge_x = left_edge[0].x();
  if (end.x() < (left_edge_x - EPSILON)) {
    double intersection_y = m * (left_edge_x - start.x()) + start.y();
    if (intersection_y >= bottom_edge[0].y() && intersection_y <= top_edge[0].y()) {
      ret.set_has_intersection(true);
      ret.set_intersection_point(Vector2d(left_edge_x, intersection_y));
      if (intersection_y >= from_diamonds(4 - 0.25) || intersection_y <= from_diamonds(0.25)) {
        ret.set_possible(false);
      }
      ret.set_intersection_edge(Edge::LEFT);
      double scaling_factor = rail_path_scaling_factor_given_absolute_angle_to_edge(absolute_angle_to_edge(angle, Edge::LEFT));
      ret.set_end_vector(reflect_point_in_y_axis(Vector2d(end - ret.get_intersection_point())) * scaling_factor);
      ret.set_scaling_factor(scaling_factor);
      return ret;
    }
  }
  // Right side
  double right_edge_x = right_edge[0].x();
  if (end.x() > (right_edge_x + EPSILON)) {
    double intersection_y = m * (right_edge_x - start.x()) + start.y();
    if (intersection_y >= bottom_edge[0].y() && intersection_y <= top_edge[0].y()) {
      ret.set_has_intersection(true);
      ret.set_intersection_point(Vector2d(right_edge_x, intersection_y));
      if (intersection_y >= from_diamonds(4 - 0.25) || intersection_y <= from_diamonds(0.25)) {
        ret.set_possible(false);
      }
      ret.set_intersection_edge(Edge::RIGHT);
      double scaling_factor = rail_path_scaling_factor_given_absolute_angle_to_edge(absolute_angle_to_edge(angle, Edge::RIGHT));
      ret.set_end_vector(reflect_point_in_y_axis(Vector2d(end - ret.get_intersection_point())) * scaling_factor);
      ret.set_scaling_factor(scaling_factor);
      return ret;
    }
  }
  // Bottom side
  double bottom_edge_y = bottom_edge[0].y();
  if (end.y() < (bottom_edge_y - EPSILON)) {
    double intersection_x = (bottom_edge_y - start.y()) / m + start.x();
    if (intersection_x >= left_edge[0].x() && intersection_x <= right_edge[0].x()) {
      ret.set_has_intersection(true);
      ret.set_intersection_point(Vector2d(intersection_x, bottom_edge_y));
      if (intersection_x >= from_diamonds(8 - 0.25) || intersection_x <= from_diamonds(0.25) || (intersection_x >= from_diamonds(4 - 0.25) && intersection_x <= from_diamonds(4 + 0.25))) {
        ret.set_possible(false);
      }
      ret.set_intersection_edge(Edge::BOTTOM);
      double scaling_factor = rail_path_scaling_factor_given_absolute_angle_to_edge(absolute_angle_to_edge(angle, Edge::BOTTOM));
      ret.set_end_vector(reflect_point_in_x_axis(Vector2d(end - ret.get_intersection_point())) * scaling_factor);
      ret.set_scaling_factor(scaling_factor);
      return ret;
    }
  }
  // Top side
  double top_edge_y = top_edge[0].y();
  if (end.y() > (top_edge_y + EPSILON)) {
    double intersection_x = (top_edge_y - start.y()) / m + start.x();
    if (intersection_x >= left_edge[0].x() && intersection_x <= right_edge[0].x()) {
      ret.set_has_intersection(true);
      ret.set_intersection_point(Vector2d(intersection_x, top_edge_y));
      if (intersection_x >= from_diamonds(8 - 0.25) || intersection_x <= from_diamonds(0.25) || (intersection_x >= from_diamonds(4 - 0.25) && intersection_x <= from_diamonds(4 + 0.25))) {
        ret.set_possible(false);
      }
      ret.set_intersection_edge(Edge::TOP);
      double scaling_factor = rail_path_scaling_factor_given_absolute_angle_to_edge(absolute_angle_to_edge(angle, Edge::TOP));
      ret.set_end_vector(reflect_point_in_x_axis(Vector2d(end - ret.get_intersection_point())) * scaling_factor);
      ret.set_scaling_factor(scaling_factor);
      return ret;
    }
  }
  ret.set_has_intersection(false);
  return ret;
}

double square(double num) {
  return num * num;
}

// All calculations in meters.
// TODO: Tailor granularities better.
double strength_to_speed(short strength)
{
  double speed_interval = MAX_CUE_BALL_SPEED - MIN_CUE_BALL_SPEED;
  double multiplier = square(strength) / square(NUM_STRENGTHS - 1);
  return MIN_CUE_BALL_SPEED + speed_interval * multiplier;
}

Speed speed_to_speed_type(double speed)
{
  return speed < 1.5 ? SLOW : FAST;
}

// All calculations in meters.
double get_angular_speed(double cue_ball_speed, Spin spin) {
  switch (spin) {
    case STUN:
      return 0;
    case LIGHT_FOLLOW:
      return -0.7 * cue_ball_speed / BALL_RADIUS_IN_METERS;
    case LIGHT_DRAW:
      return 0.7 * cue_ball_speed / BALL_RADIUS_IN_METERS;
    case HEAVY_FOLLOW:
      return -1.25 * cue_ball_speed / BALL_RADIUS_IN_METERS;
    case HEAVY_DRAW:
      return 1.25 * cue_ball_speed / BALL_RADIUS_IN_METERS;
    case MAX_NUM_SPINS:
      throw std::invalid_argument( "Max spin invalid" );
  }
  throw std::invalid_argument( "Invalid spin" );
}

// All calculations in meters.
// TODO: Go over this function. Calculation might be off.
// TODO: Unit test this.
vector<Vector2d> get_cue_ball_path(const Shot& shot, short strength, Spin spin) {
  // The bulk of these calculations are in meters per second. They are converted to units at the last second.
  vector<Vector2d> ret;
  double phi = shot.get_shot_angle();
  // This is the multiplier to apply to the x value of the trajectory vectors. The calculations
  // below assume that the object ball travels in a direction counterclockwise from the cue ball
  // shot path. If this is not the case (phi < 0), then we must multiply the resulting x position
  // of the trajectories by negative 1.
  double x_multiplier = phi < 0 ? -1 : 1;
  phi = abs(phi);
  double v = strength_to_speed(strength);
  double w = get_angular_speed(v, spin);
  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double sin_phi_plus_R_times_w_over_v_sin_phi = sin_phi + BALL_RADIUS_IN_METERS * w / (v * sin_phi);
  if (phi == 0) {
    sin_phi_plus_R_times_w_over_v_sin_phi = sin_phi;
  }
  double root_portion = sqrt(square(cos_phi) + square(sin_phi_plus_R_times_w_over_v_sin_phi));
  double u_times_g = U * G;
  double delta_t = 2 * v * sin_phi * root_portion / (7 * u_times_g);
  double delta_t_squared = square(delta_t);

  double angle_from_vertical_vector_to_cue_ball_aiming_line =
    angle_between_vectors(Vector2d(0, 1), shot.get_cue_ball_to_ghost_ball());
  Rotation2Dd rotation(angle_from_vertical_vector_to_cue_ball_aiming_line);
  // x_c_after_curved_trajectory and y_c_after_curved_trajectory are calculated as in
  // https://billiards.colostate.edu/technical_proofs/new/TP_A-4.pdf.
  // They represent the direction of travel of the cue ball after contact with the object ball
  // assuming the cue ball was originally traveling upwards along the y-axis.
  Vector2d curved_trajectory_vector = Vector2d(
    x_multiplier * (v * delta_t * sin_phi * cos_phi - u_times_g * delta_t_squared * cos_phi / (2 * root_portion)),
    v * delta_t * square(sin_phi) - u_times_g * delta_t_squared * sin_phi_plus_R_times_w_over_v_sin_phi / (2 * root_portion));
  curved_trajectory_vector = rotation * curved_trajectory_vector;
  curved_trajectory_vector *= DIAMONDS_PER_METER * UNITS_PER_DIAMOND;

  Vector2d final_velocity_vector = Vector2d(
    x_multiplier * 5 * v * sin_phi * cos_phi / 7,
    (5 * v * square(sin_phi) - 2 * BALL_RADIUS_IN_METERS * w) / 7);
  double roll_distance = speed_to_distance(final_velocity_vector.norm());
  Vector2d final_trajectory_vector = final_velocity_vector.normalized() * roll_distance;
  final_trajectory_vector = rotation * final_trajectory_vector;
  Vector2d cue_ball_position = shot.get_ghost_ball()->get_coords();
  ret.push_back(cue_ball_position);
  ret.push_back(curved_trajectory_vector);
  ret.push_back(final_trajectory_vector);
  return ret;
}

double speed_to_distance(double speed) {
  return square(speed) * DIAMONDS_PER_METER * UNITS_PER_DIAMOND / (2 * U * G);
}

bool get_object_ball_shot_possible_from_object_ball_speed(double object_ball_speed, const Vector2d& ball, const Pocket& pocket) {
  double distance = speed_to_distance(object_ball_speed);
  return (pocket.get_position() - ball).norm() <= distance;
}

// TODO: Possibly wrong.
bool get_object_ball_shot_possible_from_cue_ball_speed(double cue_ball_speed, double shot_angle, const Vector2d& ball, const Pocket& pocket) {
  return get_object_ball_shot_possible_from_object_ball_speed(cue_ball_speed_to_object_ball_speed(cue_ball_speed, shot_angle), ball, pocket);
}

double cue_ball_speed_to_object_ball_speed(double speed, double shot_angle) {
  return 5 * speed * cos(shot_angle) / 7;
}


/**
 * Populates the cue ball paths after a shot is taken.
 * TODO: Unit test this.
 */
void populate_post_shot_table()
{
  post_shot_table =
    vector<vector<vector<vector<vector<vector<PostShot>>>>>>(
      LENGTH + 1, vector<vector<vector<vector<vector<PostShot>>>>>(
        WIDTH + 1, vector<vector<vector<vector<PostShot>>>>(
          player_balls.size(), vector<vector<vector<PostShot>>>(
            pockets.size(), vector<vector<PostShot>>(
              NUM_STRENGTHS, vector<PostShot>(
                Spin::MAX_NUM_SPINS))))));
  for (unsigned short l = 0; l <= LENGTH; ++l)
  {
    for (unsigned short w = 0; w <= WIDTH; ++w)
    {
      for (unsigned short o = 0; o < player_balls.size(); ++o)
      {
        for (unsigned short p = 0; p < pockets.size(); ++p)
        {
          const Shot &shot = shot_table[l][w][o][p];
          for (unsigned short st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned short sp = 0; sp < Spin::MAX_NUM_SPINS; ++sp)
            {
              PostShot &post_shot = post_shot_table[l][w][o][p][st][sp];
              post_shot.set_possible();
              post_shot.set_shot(&shot);
              if (!shot.get_possible())
              {
                post_shot.set_impossible();
                continue;
              }
              double cue_ball_speed = strength_to_speed(st);
              if (!get_object_ball_shot_possible_from_cue_ball_speed(cue_ball_speed, shot.get_shot_angle(), player_balls[o], pockets[p])) {
                post_shot.set_impossible();
                continue;
              }
              post_shot.set_speed_type(speed_to_speed_type(cue_ball_speed));
              vector<Vector2d> cue_ball_path = get_path_with_reflections(get_cue_ball_path(shot, st, (Spin) sp));
              if (cue_ball_path.size() == 0) {
                post_shot.set_impossible();
                continue;
              }
              post_shot.set_cue_ball_path(cue_ball_path);
              for (unsigned int i = 1; i < cue_ball_path.size(); ++i) {
                BallObstructions cue_ball_path_obstructions =
                  get_obstructions_on_ball_path_for_ball_index(o, cue_ball_path[i - 1], cue_ball_path[i], true);
                if (cue_ball_path_obstructions.get_has_permanent_obstruction())
                {
                  post_shot.set_impossible();
                  break;
                }
                post_shot.add_player_balls_to_obstructions(cue_ball_path_obstructions.get_obstructing_player_balls());
              }
              if(!post_shot.get_possible()) {
                continue;
              }
              switch(post_shot.get_speed_type()) {
                case FAST:
                  post_shot.set_shot_difficulty(shot.get_fast_shot_difficulty());
                  post_shot.set_weighted_shot_difficulty(shot.get_weighted_fast_shot_difficulty());
                  break;
                case SLOW:
                  post_shot.set_shot_difficulty(shot.get_slow_shot_difficulty());
                  post_shot.set_weighted_shot_difficulty(shot.get_weighted_slow_shot_difficulty());
                  break;
              }
              post_shot.add_player_balls_to_obstructions(shot.get_shot_obstructions().get_obstructing_player_balls());
            }
          }
        }
      }
    }
  }
}

double get_weighted_shot_difficulty(double shot_difficulty) {
  return shot_difficulty * shot_difficulty;
}

short radians_to_decidegrees(double radians) {
  return rint(180 * 10 * radians / M_PI);
}

double radians_to_degrees(double radians) {
  return 180 * radians / M_PI;
}

double degrees_to_radians(double degrees) {
  return degrees * M_PI / 180;
}

double angle_between_vectors(const Vector2d& vec1, const Vector2d& vec2) {
  return atan2(vec1.x() * vec2.y() - vec1.y() * vec2.x(), vec1.x() * vec2.x() + vec1.y() * vec2.y());
}

double get_shot_angle(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Pocket& pocket) {
  Vector2d ghost_ball_to_pocket = pocket.get_position() - ghost_ball;
  Vector2d cue_ball_to_ghost_ball = ghost_ball - cue_ball;
  return angle_between_vectors(cue_ball_to_ghost_ball, ghost_ball_to_pocket);
}

double solve_quadratic_equation(double a, double b, double c) {
  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    return -1;
  }
  return (-1 * b + sqrt(discriminant)) / (2 * a);
}

double get_effective_pocket_size(const Vector2d &ghost_ball, const Pocket& pocket, Speed speed) {
  Vector2d pocket_to_ghost_ball = ghost_ball - pocket.get_position();
  // For effective pocket size calculations we do not need the angle to be directed.
  short deciangle =
    radians_to_decidegrees(abs(angle_between_vectors(pocket_to_ghost_ball, pocket.get_center_line())));
  const map<short, double>& deciangle_to_effective_pocket_size_map =
    speed == Speed::SLOW ?
      (pocket.get_type() == Pocket::PocketType::SIDE ?
      deciangle_to_effective_pocket_size_slow_side :
      deciangle_to_effective_pocket_size_slow_corner) :
      (pocket.get_type() == Pocket::PocketType::SIDE ?
      deciangle_to_effective_pocket_size_fast_side :
      deciangle_to_effective_pocket_size_fast_corner);

  return from_diamonds(deciangle_to_effective_pocket_size(deciangle, deciangle_to_effective_pocket_size_map));
}

double get_margin_of_error_for_shot(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Pocket& pocket, Speed speed)
{
  Vector2d pocket_to_ghost_ball = ghost_ball - pocket.get_position();
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  // The shot becomes difficult if the cue ball is too close to the ghost ball.
  if (ghost_ball_to_cue_ball.norm() < from_diamonds(0.25)) {
    return 0;
  }
  double s = get_effective_pocket_size(ghost_ball, pocket, speed);
  double dp = pocket_to_ghost_ball.norm();
  double theta = 2 * atan(s / (2 * dp));
  double d = ghost_ball_to_cue_ball.norm();
  double phi = abs(get_shot_angle(cue_ball, ghost_ball, pocket));
  double theta_minus_phi = theta - phi;
  double sin_phi = sin(phi);
  double sin_theta_minus_phi = sin(theta_minus_phi);
  double quadratic_a = BALL_RADIUS * sin_theta_minus_phi + BALL_RADIUS * sin_phi;
  double quadratic_b = 2 * BALL_RADIUS * cos(phi) + d - 2 * BALL_RADIUS * cos(theta_minus_phi);
  double quadratic_c = -2 * BALL_RADIUS * sin_phi - 2 * BALL_RADIUS * sin_theta_minus_phi;

  double ret = solve_quadratic_equation(quadratic_a, quadratic_b, quadratic_c);
  if (ret < 0) {
    return 0;
  }
  return ret;
}

double get_shot_difficulty(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Pocket& pocket, Speed speed) {
  return 1 / get_margin_of_error_for_shot(cue_ball, ghost_ball, pocket, speed);
}

void populate_shot_table_difficulty()
{
  for (unsigned short o = 0; o < player_balls.size(); ++o)
  {
    for (unsigned short p = 0; p < pockets.size(); ++p)
    {
      for (unsigned short l = 0; l <= LENGTH; ++l)
      {
        for (unsigned short w = 0; w <= WIDTH; ++w)
        {
          Shot &shot = shot_table[l][w][o][p];
          const GhostBall& ghost_ball = *shot.get_ghost_ball();
          if (!shot.get_possible()) {
            continue;
          }
          Vector2d cue_ball = move_ball_in_from_rails(Vector2d(l, w));
          shot.set_shot_angle(get_shot_angle(cue_ball, ghost_ball.get_coords(), pockets[p]));
          shot.set_cue_ball_to_ghost_ball(ghost_ball.get_coords() - cue_ball);
          double slow_shot_difficulty = get_shot_difficulty(cue_ball, ghost_ball.get_coords(), pockets[p], Speed::SLOW);
          if (slow_shot_difficulty == std::numeric_limits<double>::infinity())
          {
            shot.set_impossible();
            continue;
          }
          shot.set_slow_shot_difficulty(slow_shot_difficulty);
          shot.set_weighted_slow_shot_difficulty(get_weighted_shot_difficulty(slow_shot_difficulty));
          double fast_shot_difficulty = get_shot_difficulty(cue_ball, ghost_ball.get_coords(), pockets[p], Speed::FAST);
          shot.set_fast_shot_difficulty(fast_shot_difficulty);
          shot.set_weighted_fast_shot_difficulty(get_weighted_shot_difficulty(fast_shot_difficulty));
        }
      }
    }
  }
}

Vector2d move_ball_in_from_rails(const Vector2d& position) {
  double x_position_in_units = position.x();
  double y_position_in_units = position.y();
  if (x_position_in_units < BALL_RADIUS) {
    x_position_in_units = BALL_RADIUS;
  } else if (x_position_in_units > LENGTH - BALL_RADIUS) {
    x_position_in_units = LENGTH - BALL_RADIUS;
  }
  if (y_position_in_units < BALL_RADIUS) {
    y_position_in_units = BALL_RADIUS;
  } else if (y_position_in_units > WIDTH - BALL_RADIUS) {
    y_position_in_units = WIDTH - BALL_RADIUS;
  }
  return Vector2d(x_position_in_units, y_position_in_units);
}

void populate_shot_table_obstructions()
{
  shot_table =
      vector<vector<vector<vector<Shot>>>>(
          LENGTH + 1, vector<vector<vector<Shot>>>(
              WIDTH + 1, vector<vector<Shot>>(
                  player_balls.size(), vector<Shot>(pockets.size()))));
  for (unsigned short o = 0; o < player_balls.size(); ++o)
  {
    for (unsigned short p = 0; p < pockets.size(); ++p)
    {
      const BallObstructions& player_ball_to_pocket_obstructions = player_ball_to_pocket_obstructions_table[o][p];
      const GhostBall* ghost_ball_ptr = &ghost_ball_position_table[o][p];
      for (unsigned short l = 0; l <= LENGTH; ++l)
      {
        for (unsigned short w = 0; w <= WIDTH; ++w)
        {
          Shot &shot = shot_table[l][w][o][p];
          shot.set_possible();

          if (player_ball_to_pocket_obstructions.get_has_permanent_obstruction()) {
            shot.set_impossible();
            continue;
          }
          shot.set_shot_obstructions(player_ball_to_pocket_obstructions);
          if (!ghost_ball_ptr->get_possible())
          {
            shot.set_impossible();
            continue;
          }
          shot.set_ghost_ball(ghost_ball_ptr);

          Vector2d cue_ball = move_ball_in_from_rails(Vector2d(l, w));
          BallObstructions cue_to_object_obstructions =
              get_obstructions_on_ball_path_for_ball_index(o, cue_ball, shot.get_ghost_ball()->get_coords(), false);
          if (cue_to_object_obstructions.get_has_permanent_obstruction())
          {
            shot.set_impossible();
            continue;
          }
          shot.add_player_balls_to_shot_obstructions(cue_to_object_obstructions.get_obstructing_player_balls());
        }
      }
    }
  }
}

GhostBall get_ghost_ball_for_shot(const Vector2d &ball, const Pocket &pocket)
{
  GhostBall ghost_ball;
  Vector2d pocket_to_ball = ball - pocket.get_position();
  Vector2d coords = ball + pocket_to_ball.normalized() * BALL_DIAMETER;
  if (coords.x() < left_edge[0].x() || coords.x() > right_edge[0].x() ||
      coords.y() < bottom_edge[0].y() || coords.y() > top_edge[0].y())
  {
    ghost_ball.set_impossible();
  }
  else
  {
    ghost_ball.set_coords(coords);
  }
  return ghost_ball;
}

void populate_ghost_ball_position_table()
{
  ghost_ball_position_table =
      vector<vector<GhostBall>>(player_balls.size(), vector<GhostBall>(pockets.size()));
  for (unsigned short p = 0; p < pockets.size(); ++p)
  {
    for (unsigned short o = 0; o < player_balls.size(); ++o)
    {
      ghost_ball_position_table[o][p] = get_ghost_ball_for_shot(player_balls[o], pockets[p]);
    }
  }
}

double distance_from_point_to_segment(const Vector2d &p, const Vector2d &segment_start, const Vector2d &segment_end)
{
  Vector2d start_to_point = p - segment_start;
  Vector2d start_to_end = segment_end - segment_start;
  const double dot = start_to_point.dot(start_to_end);
  if (dot < 0)
  {
    return numeric_limits<double>::infinity();
  }
  const double start_to_end_length_squared = start_to_end.squaredNorm();
  if (dot > start_to_end_length_squared)
  {
    return numeric_limits<double>::infinity();
  }
  // t is the ratio of the distance of (start, projected point) to
  // (start, end).
  const double t = dot / start_to_end_length_squared;
  const Vector2d projected_point = segment_start + t * start_to_end;
  return (p - projected_point).norm();
}

SegmentRange get_segment_range(const Vector2d &segment_start, const Vector2d &segment_end)
{
  SegmentRange segment_range;
  if (segment_start.x() < segment_end.x())
  {
    segment_range.min_x = segment_start.x();
    segment_range.max_x = segment_end.x();
  }
  else
  {
    segment_range.min_x = segment_end.x();
    segment_range.max_x = segment_start.x();
  }
  if (segment_start.y() < segment_end.y())
  {
    segment_range.min_y = segment_start.y();
    segment_range.max_y = segment_end.y();
  }
  else
  {
    segment_range.min_y = segment_end.y();
    segment_range.max_y = segment_start.y();
  }
  return segment_range;
}

bool get_ball_intersects_ball_path(const Vector2d &ball, const Vector2d &ball_path_start, const Vector2d &ball_path_end)
{
  double ball_x = ball.x();
  double ball_y = ball.y();
  SegmentRange segment_range = get_segment_range(ball_path_start, ball_path_end);

  if (ball_x < segment_range.min_x - BALL_DIAMETER || ball_x > segment_range.max_x + BALL_DIAMETER)
  {
    return false;
  }
  if (ball_y < segment_range.min_y - BALL_DIAMETER || ball_y > segment_range.max_y + BALL_DIAMETER)
  {
    return false;
  }
  double distance_from_ball_to_start = (ball_path_start - ball).norm();
  double distance_from_ball_to_end = (ball_path_end - ball).norm();
  if (distance_from_ball_to_start < BALL_DIAMETER || distance_from_ball_to_end < BALL_DIAMETER)
  {
    return true;
  }
  return distance_from_point_to_segment(ball, ball_path_start, ball_path_end) < BALL_DIAMETER;
}

short eight_ball_index()
{
  return 0;
}

BallObstructions get_obstructions_on_ball_path_for_ball_index(
    short object_ball_index,
    const Vector2d &ball_path_start,
    const Vector2d &ball_path_end,
    bool is_cue_ball_after_contact)
{
  BallObstructions obstructions;
  for (unsigned short o = 0; o < opponent_balls.size(); ++o)
  {
    if (get_ball_intersects_ball_path(opponent_balls[o], ball_path_start, ball_path_end))
    {
      // On the 8 ball shot, if {@code is_cue_ball_after_contact} is set, contacting an opponent
      // ball is actually a good thing. It will prevent the cue ball from scratching in a pocket.
      // All other times, we do not wish to contact opponent object balls if possible.
      if (is_cue_ball_after_contact && object_ball_index == eight_ball_index())
      {
        return obstructions;
      }
      obstructions.set_has_permanent_obstruction();
      return obstructions;
    }
  }
  if (is_cue_ball_after_contact)
  {
    for (unsigned short p = 0; p < pockets.size(); ++p)
    {
      if (get_ball_intersects_ball_path(pockets[p].get_position(), ball_path_start, ball_path_end))
      {
        obstructions.set_has_permanent_obstruction();
        return obstructions;
      }
    }
  }
  if (object_ball_index == eight_ball_index())
  {
    return obstructions;
  }
  if (get_ball_intersects_ball_path(eight_ball, ball_path_start, ball_path_end))
  {
    obstructions.set_has_permanent_obstruction();
    return obstructions;
  }
  for (unsigned short i = 0; i < player_balls.size(); ++i)
  {
    if (i == object_ball_index)
    {
      continue;
    }
    bool ball_intersects_ball_path = get_ball_intersects_ball_path(player_balls[i], ball_path_start, ball_path_end);
    if (ball_intersects_ball_path) {
      if (i == eight_ball_index()) {
        obstructions.set_has_permanent_obstruction();
        return obstructions;
      }
      obstructions.add_obstructing_player_balls(i);
    }
  }
  return obstructions;
}

void populate_player_ball_to_pocket_obstructions_table()
{
  player_ball_to_pocket_obstructions_table =
      vector<vector<BallObstructions>>(player_balls.size(), vector<BallObstructions>(pockets.size()));
  for (unsigned short p = 0; p < pockets.size(); ++p)
  {
    const Vector2d& pocket = pockets[p].get_position();
    for (unsigned short b = 0; b < player_balls.size(); ++b)
    {
      player_ball_to_pocket_obstructions_table[b][p] =
          get_obstructions_on_ball_path_for_ball_index(
            b, player_balls[b], pocket, false /* is_cue_ball_after_contact */);
    }
  }
}

/**
 * Initialize the edges of the table. Relies on initializing pockets.
 */
void initialize_table_edges()
{
  bottom_edge.clear();
  bottom_edge.push_back(Vector2d(BALL_RADIUS, BALL_RADIUS));
  bottom_edge.push_back(Vector2d(LENGTH - BALL_RADIUS, BALL_RADIUS));
  right_edge.clear();
  right_edge.push_back(Vector2d(LENGTH - BALL_RADIUS, BALL_RADIUS));
  right_edge.push_back(Vector2d(LENGTH - BALL_RADIUS, WIDTH - BALL_RADIUS));
  top_edge.clear();
  top_edge.push_back(Vector2d(LENGTH - BALL_RADIUS, WIDTH - BALL_RADIUS));
  top_edge.push_back(Vector2d(BALL_RADIUS, WIDTH - BALL_RADIUS));
  left_edge.clear();
  left_edge.push_back(Vector2d(BALL_RADIUS, WIDTH - BALL_RADIUS));
  left_edge.push_back(Vector2d(BALL_RADIUS, BALL_RADIUS));
}

/**
 * Initialize the pockets as a vector of Vector2ds.
 */
void initialize_pockets()
{
  pockets.clear();
  pockets.push_back(Pocket(Vector2d(0, 0), Vector2d(1, 1), Pocket::PocketType::CORNER));
  pockets.push_back(Pocket(Vector2d(WIDTH, 0), Vector2d(0, 1), Pocket::PocketType::SIDE));
  pockets.push_back(Pocket(Vector2d(LENGTH, 0), Vector2d(-1, 1), Pocket::PocketType::CORNER));
  pockets.push_back(Pocket(Vector2d(0, WIDTH), Vector2d(1, -1), Pocket::PocketType::CORNER));
  pockets.push_back(Pocket(Vector2d(WIDTH, WIDTH), Vector2d(0, -1), Pocket::PocketType::SIDE));
  pockets.push_back(Pocket(Vector2d(LENGTH, WIDTH), Vector2d(-1, -1), Pocket::PocketType::CORNER));
}

void initialize_balls(
  const Ball& cue_ball_in_diamonds,
  const Ball& eight_ball_in_diamonds,
  const vector<Ball>& player_balls_in_diamonds,
  const vector<Ball>& opponent_balls_in_diamonds) {
  cue_ball = move_ball_in_from_rails(cue_ball_in_diamonds.to_vec2d());
  eight_ball = move_ball_in_from_rails(eight_ball_in_diamonds.to_vec2d());
  player_balls.clear();
  player_balls_without_eight_ball.clear();
  player_balls.push_back(eight_ball);
  for (auto player_ball_in_diamonds : player_balls_in_diamonds) {
    Vector2d repositioned_ball = move_ball_in_from_rails(player_ball_in_diamonds.to_vec2d());
    player_balls.push_back(repositioned_ball);
    player_balls_without_eight_ball.push_back(repositioned_ball);
  }
  opponent_balls.clear();
  for (auto opponent_ball_in_diamonds : opponent_balls_in_diamonds) {
    opponent_balls.push_back(move_ball_in_from_rails(opponent_ball_in_diamonds.to_vec2d()));
  }
}

void initialize(
  const Ball& cue_ball_in_diamonds,
  const Ball& eight_ball_in_diamonds,
  const vector<Ball>& player_balls_in_diamonds,
  const vector<Ball>& opponent_balls_in_diamonds) {
  initialize_pockets();
  initialize_table_edges();
  initialize_balls(
    cue_ball_in_diamonds,
    eight_ball_in_diamonds,
    player_balls_in_diamonds,
    opponent_balls_in_diamonds);
}

void populate_tables() {
  auto start = chrono::system_clock::now();

  populate_player_ball_to_pocket_obstructions_table();
  auto end = chrono::system_clock::now();
  chrono::duration<double> elapsed_seconds = end - start;
  cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  populate_ghost_ball_position_table();
  end = chrono::system_clock::now();
  elapsed_seconds = end - start;
  cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  populate_shot_table_obstructions();
  end = chrono::system_clock::now();
  elapsed_seconds = end - start;
  cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  populate_shot_table_difficulty();
  end = chrono::system_clock::now();
  elapsed_seconds = end - start;
  cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  populate_post_shot_table();
  end = chrono::system_clock::now();
  elapsed_seconds = end - start;
  cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  populate_selected_shot_table();
  end = chrono::system_clock::now();
  elapsed_seconds = end - start;
  cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}