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
/**
 * Represents information about the remaining runout after a shot is taken.
 */
struct remaining_runout_struct
{
  Vector2d cue_ball_location_considered;

  float difficulty;

  unsigned short int next_object_ball;
};

/**
 * Whether or not a shot is obstructed and whether it can become unobstructed.
 */
struct obstructions_struct
{
  /**
   * A shot is permanently obstructed if blocked by the 8 ball or opponent object balls.
   */
  bool has_permanent_obstruction = true;
  /**
   * Obstructing object balls belonging to the player are listed here.
   * Only accurately populated if has_permanent_obstruction is false.
   */
  set<unsigned short int> obstructing_object_balls;
};

/**
 * Holds information about ghost balls.
 */
struct ghost_ball_struct
{
  /**
   * Whether there is a valid ghost ball or not.
   */
  bool possible = false;
  /**
   * The coordinates.
   */
  Vector2d coords;
};

/**
 * The possible information with regards to a shot other than cue ball travel path after the shot.
 */
struct shot_info_struct
{
  bool possible = true;
  const ghost_ball_struct *ghost_ball;
  const obstructions_struct *ball_to_pocket_obstructions;
  /**
   * The difficulty of the shot. Does not take obstructions into account. Always populated.
   */
  float difficulty = std::numeric_limits<float>::infinity();
  /**
   * The weighted difficulty of the shot so that we can just add numbers together when considering runout paths.
   * Always populated.
   * TODO: Make this weighting more accurate.
   */
  float weighted_difficulty = std::numeric_limits<float>::infinity();
  /**
   * The obstructions for this shot. Includes balls obstructing the cue ball to the
   * ghost ball path as well as the obstructions from the object ball to the pocket.
   * Always populated.
   */
  obstructions_struct shot_obstructions;
};

/**
 * The information about the cue ball path after a shot. To determine whether a shot is plausible or not we must
 * consider both this struct and the shot_info_struct. Only populated if shot info is possible.
 */
struct shot_path_struct
{
  bool possible = false;
  /**
   * The line segments that make up the shot path.
   */
  vector<Vector2d> path_segments;
  /**
   * The obstructions along this path.
   */
  obstructions_struct shot_obstructions;
  /**
   * The final location of the cue ball along this path.
   */
  Vector2d final_position;
  /**
   * The difficulty of the shot. Does not take obstructions into account. Always populated.
   */
  float difficulty = std::numeric_limits<float>::infinity();
};

/**
 * The information about both the shot info and shot path.
 */
struct shot_info_and_path_struct
{
  const shot_info_struct *shot_info;
  const shot_path_struct *shot_path;
  bool possible = true;
  float difficulty = std::numeric_limits<float>::infinity();
  float weighted_difficulty = std::numeric_limits<float>::infinity();
};

/**
 * Holds information about the intersection of two segments.
 */
struct segment_intersection_struct
{
  /**
   * Whether or not there is an intersection between the line segments.
   */
  bool has_intersection = false;
  /**
   * The coordinates of the intersection. Only populated if has_intersection is false.
   */
  Vector2d intersection_point;
};

/**
 * Holds information about the minimum and maximum values of line segments.
 */
struct segment_range_struct
{
  /**
   * The minimum x coordinate of the segments given.
   */
  float min_x = 0;
  /**
   * The maximum x coordinate of the segments given.
   */
  float max_x = 0;
  /**
   * The minimum y coordinate of the segments given.
   */
  float min_y = 0;
  /**
   * The maximum y coordinate of the segments given.
   */
  float max_y = 0;
};

/**
 * Holds coordinate information in unsigned short ints.
 */
struct unsigned_short_int_coordinates_struct
{
  /**
   * The x coordinate.
   */
  unsigned short int x = 0;
  /**
   * The y coordinate.
   */
  unsigned short int y = 0;
};

/**
 * Holds information about a ball path after reflection off a single rail of a pool table.
 */
struct single_rail_reflection_struct
{
  /**
   * Whether or not the ball was reflected off this rail.
   */
  bool has_reflection = false;
  /**
   * The intersection point with the rail.
   * Only populated if has_reflection is true.
   */
  Vector2d intersection_point;
  /**
   * After reflection off the rail, this is the new end point of the cue ball.
   * Only populated if has_reflection is true.
   */
  Vector2d end_point;
  /**
   * The distance the ball traveled along this path after hitting the rail to get to the end point.
   * Only populated if has_reflection is true.
   */
  float distance_traveled_after_rail;
};

/**
 * Information about shot angles such as the tangent line.
 */
struct shot_angle_struct
{
  enum FollowDirection
  {
    CLOCKWISE,
    COUNTER_CLOCKWISE
  };
  // The direction from the tangent line for the path if the cue ball is hit with follow.
  FollowDirection follow_direction;
  // The tangent line direction. It is a unit vector.
  Vector2d tangent_line_direction;
  // The origin point of the tangent line. Usually equal to the ghost ball contact point.
  Vector2d origin;
  // The pocket direction. It is a unit vector.
  Vector2d pocket_direction;
  // The proportion of the initial cue ball distance that the cue ball will travel at after contact.
  float fractional_distance;
  // The cue angle of the shot.
  float cut_angle_in_radians;
  /**
   * The minimum travel distance of a stunned cue ball after contact with the ghost ball.
   */
  float minimum_travel_distance_of_cue_ball;
  Vector2d cue_ball_to_ghost_ball;
};

/**
 * Information about the selected shot.
 */
struct selected_shot_struct
{
  /**
   * Whether the shot is possible or not.
   */
  bool possible = false;
  /**
   * The strength of the shot. (0..NUM_STRENGTHS)
   * Only populated if possible is true.
   */
  unsigned short int strength;
  /**
   * The spin of the shot. (0..NUM_SPINS)
   * Only populated if possible is true.
   */
  unsigned short int spin;
  /**
   * The index of the object ball to shoot. The eight ball is at object_balls.size().
   * Only populated if possible is true.
   */
  unsigned short int object_ball;
  /**
   * Which pocket index to shoot the ball into.
   * Only populated if possible is true.
   */
  unsigned short int pocket;
  /**
   * The total weighted difficulty of the runout starting from this ball.
   * Only populated if possible is true.
   */
  float total_weighted_difficulty = std::numeric_limits<float>::infinity();
  float current_weighted_difficulty = std::numeric_limits<float>::infinity();
  /**
   * The points defining the line segments the cue ball travels along after contact with the object ball.
   * Only populated if possible is true.
   */
  vector<Vector2d> path_segments;
  /**
   * Expected cue ball final position
   */
  Vector2d expected_cue_ball;
  /**
   * Missed cue ball position 1.
   */
  Vector2d missed_cue_ball_1;
  /**
   * Missed cue ball position 2.
   */
  Vector2d missed_cue_ball_2;
  /**
   * Expected cue ball position's next object ball shot.
   */
  unsigned short int expected_object_ball;
  /**
   * Missed cue ball position 1's next object ball shot.
   */
  unsigned short int missed_1_object_ball;
  /**
   * Missed cue ball position 2's next object ball shot.
   */
  unsigned short int missed_2_object_ball;
  /**
   * The next x-coordinate of the cue ball after this shot.
   * Only populated if possible is true.
   */
  unsigned short int next_x;
  /**
   * The next y-coordinate of the cue ball after this shot.
   * Only populated if possible is true.
   */
  unsigned short int next_y;
  /**
   * The combination of object balls for the next shot, if any.
   * Only populated if possible is true.
   */
  unsigned short int next_combo;
};

/**
 * The name of the file to output game data.
 */
const string GAME_DATA_FILE = "game_data.js";

/**
 * The length of a diamond in units.
 */
const unsigned short int UNITS_PER_DIAMOND = 4;
/**
 * The number of shot strengths to consider. If this is not 12, strength_to_distance must be changed.
 */
const unsigned short int NUM_STRENGTHS = 12;
/**
 * The number of shot spins to consider.
 */
const unsigned short int NUM_SPINS = 5;
/**
 * The maximum difficulty of individual shot to consider shooting.
 */
const unsigned int MAX_DIFFICULTY_SHOT_TO_CONSIDER = 5 * UNITS_PER_DIAMOND * 5 * UNITS_PER_DIAMOND * 1.33;
/**
 * The unit diameter of the balls.
 */
const float BALL_DIAMETER = 0.18 * UNITS_PER_DIAMOND;
const float BALL_RADIUS = BALL_DIAMETER / 2;
/**
 * The width of the pool table in units.
 */
const unsigned short int WIDTH = UNITS_PER_DIAMOND * 4;
/**
 * The length of the pool table in units.
 */
const unsigned short int LENGTH = WIDTH * 2;
/**
 * The maximum number of diamonds the cue ball can travel if untouched by an object ball.
 */
const unsigned short int MAX_DIAMONDS = 30;
bool ball_in_hand = false;
/**
 * The positions of the pockets.
 */
vector<Vector2d> pockets;
/**
 * The position of the cue ball.
 */
Vector2d cue_ball;
/**
 * The position of the eight ball.
 */
Vector2d eight_ball;
/**
 * The position of our object balls.
 */
vector<Vector2d> object_balls;
/**
 * The position of our opponent's object balls.
 */
vector<Vector2d> opponent_object_balls;
/**
 * The bottom edge of the pool table.
 */
vector<Vector2d> bottom_edge;
/**
 * The right edge of the pool table.
 */
vector<Vector2d> right_edge;
/**
 * The top edge of the pool table.
 */
vector<Vector2d> top_edge;
/**
 * The left edge of the pool table.
 */
vector<Vector2d> left_edge;

/**
 * A table of obstructions from each object ball index to a given pocket. Includes the 8 ball at index
 * object_balls.size(). This is only used to help populate shot_obstructions.
 * Only modified to populate the shot info table.
 * Dimensions are [Object balls Size + 1][Pockets]
 */
vector<vector<obstructions_struct>> ball_to_pocket_obstructions_table;

/**
 * A table of ghost ball positions for a given object ball index to a pocket. Includes the 8 ball at index
 * object_balls.size().
 * Only modified to populate the shot info table.
 * Dimensions are [Object balls Size + 1][Pockets]
 */
vector<vector<ghost_ball_struct>> ghost_ball_position_table;

/**
 * A table of shot infos including obstruction information and difficulty. Includes the 8 ball at index
 * object_balls.size().
 * Dimensions are [Width + 1][Length + 1][Object balls Size + 1][Pockets]
 */
vector<vector<vector<vector<shot_info_struct>>>> shot_info_table;

/**
 * A table of possible paths given a cue ball, object ball, pocket, strength, and spin. Includes the 8 ball at index
 * object_balls.size().
 * Dimensions are [Width + 1][Length + 1][Object balls Size + 1][Pockets][Strength][Spin].
 */
vector<vector<vector<vector<vector<vector<shot_path_struct>>>>>> shot_path_table;

/**
 * A table of shot info and shot path.
 * object_balls.size().
 * Dimensions are [Width + 1][Length + 1][Object balls Size + 1][Pockets][Strength][Spin].
 */
vector<vector<vector<vector<vector<vector<shot_info_and_path_struct>>>>>> shot_info_and_path_table;

/**
 * Gets the best shot path to for each given cue ball position for the given layout.
 * Dimensions are [Width][Length][Num table layouts]. The table layout index is generated
 * by summing the (2^object ball indices) of the remaining balls. That is, if there are
 * 3 object balls, their indices will be 1,2,4. A table which consists of balls 1 and
 * 3 will be in position 5. The table with only the 8 ball remaining is at position 0.
 * This table is strange in that the 8 ball is represented at position 0 instead of object_balls.size() like all
 * other arrays.
 * Dimensions are [Width + 1][Length + 1][Num table layouts].
 */
vector<vector<vector<selected_shot_struct>>> selected_shot_table;

std::ostream &operator<<(std::ostream &o, const obstructions_struct &obstructions)
{
  o << "Permanent obstruction: " << obstructions.has_permanent_obstruction << endl;
  o << "Object ball obstructions: " << endl;
  for (unsigned short int index : obstructions.obstructing_object_balls)
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
Vector2d move_ball_in_from_rails(Vector2d position) {
  float x_position_in_units = position.x();
  float y_position_in_units = position.y();
  if (x_position_in_units < BALL_RADIUS) {
    x_position_in_units = BALL_RADIUS;
  } else if (x_position_in_units > WIDTH - BALL_RADIUS) {
    x_position_in_units = WIDTH - BALL_RADIUS;
  }
  if (y_position_in_units < BALL_RADIUS) {
    y_position_in_units = BALL_RADIUS;
  } else if (y_position_in_units > LENGTH - BALL_RADIUS) {
    y_position_in_units = LENGTH - BALL_RADIUS;
  }
  return Vector2d(x_position_in_units, y_position_in_units);
}

/**
 * Balls cannot be within BALL_RADIUS distance of the rail. Because the ball is not a point,
 * we must push them away from the rail by the ball's radius.
 */
Vector2d get_vector_from_ball_position_in_diamonds(float x_position_in_diamonds, float y_position_in_diamonds) {
  float x_position_in_units = x_position_in_diamonds * UNITS_PER_DIAMOND;
  float y_position_in_units = y_position_in_diamonds * UNITS_PER_DIAMOND;
  return move_ball_in_from_rails(Vector2d(x_position_in_units, y_position_in_units));
}

/**
 * Initialize the pockets as a vector of Vector2ds.
 */
void initialize_pockets()
{
  pockets.push_back(Vector2d(0, 0));
  pockets.push_back(Vector2d(WIDTH, 0));
  pockets.push_back(Vector2d(0, WIDTH));
  pockets.push_back(Vector2d(WIDTH, WIDTH));
  pockets.push_back(Vector2d(0, LENGTH));
  pockets.push_back(Vector2d(WIDTH, LENGTH));
}

/**
 * Initialize the edges of the table. Relies on initializing pockets.
 */
void initialize_table_edges()
{
  bottom_edge.push_back(Vector2d(BALL_RADIUS, BALL_RADIUS));
  bottom_edge.push_back(Vector2d(WIDTH - BALL_RADIUS, BALL_RADIUS));
  right_edge.push_back(Vector2d(WIDTH - BALL_RADIUS, BALL_RADIUS));
  right_edge.push_back(Vector2d(WIDTH - BALL_RADIUS, LENGTH - BALL_RADIUS));
  top_edge.push_back(Vector2d(WIDTH - BALL_RADIUS, LENGTH - BALL_RADIUS));
  top_edge.push_back(Vector2d(BALL_RADIUS, LENGTH - BALL_RADIUS));
  left_edge.push_back(Vector2d(BALL_RADIUS, LENGTH - BALL_RADIUS));
  left_edge.push_back(Vector2d(BALL_RADIUS, BALL_RADIUS));
}

/**
 * The distance from the point to the segment. Returns infinity if the perpendicular distance from the point to
 * the line defined by the segment lies off the segment. (To either side of it.)
 */
float distance_from_point_to_segment(const Vector2d &p, const Vector2d &segment_start, const Vector2d &segment_end)
{
  Vector2d start_to_point = p - segment_start;
  Vector2d start_to_end = segment_end - segment_start;
  const float dot = start_to_point.dot(start_to_end);
  if (dot < 0)
  {
    return std::numeric_limits<float>::infinity();
  }
  const float start_to_end_length_squared = start_to_end.squaredNorm();
  if (dot > start_to_end_length_squared)
  {
    return std::numeric_limits<float>::infinity();
  }
  // t is the ratio of the distance of (start, projected point) to
  // (start, end).
  const float t = dot / start_to_end_length_squared;
  const Vector2d projected_point = segment_start + t * start_to_end;
  return (p - projected_point).norm();
}

/**
 * Gets the bounding box of a given line segment.
 */
segment_range_struct get_segment_ranges(const Vector2d &segment_start, const Vector2d &segment_end)
{
  segment_range_struct segment_ranges;
  if (segment_start.x() < segment_end.x())
  {
    segment_ranges.min_x = segment_start.x();
    segment_ranges.max_x = segment_end.x();
  }
  else
  {
    segment_ranges.min_x = segment_end.x();
    segment_ranges.max_x = segment_start.x();
  }
  if (segment_start.y() < segment_end.y())
  {
    segment_ranges.min_y = segment_start.y();
    segment_ranges.max_y = segment_end.y();
  }
  else
  {
    segment_ranges.min_y = segment_end.y();
    segment_ranges.max_y = segment_start.y();
  }
  return segment_ranges;
}

/**
 * Returns whether a ball intersects the given line segment.
 */
bool ball_intersects_segment(const Vector2d &ball, const Vector2d &segment_start, const Vector2d &segment_end)
{
  float ball_x = ball.x();
  float ball_y = ball.y();
  segment_range_struct segment_ranges = get_segment_ranges(segment_start, segment_end);

  if (ball_x < segment_ranges.min_x - BALL_DIAMETER || ball_x > segment_ranges.max_x + BALL_DIAMETER)
  {
    return false;
  }
  if (ball_y < segment_ranges.min_y - BALL_DIAMETER || ball_y > segment_ranges.max_y + BALL_DIAMETER)
  {
    return false;
  }
  float distance_from_ball_to_start = (segment_start - ball).norm();
  float distance_from_ball_to_end = (segment_end - ball).norm();
  if (distance_from_ball_to_start < BALL_DIAMETER || distance_from_ball_to_end < BALL_DIAMETER)
  {
    return true;
  }
  float distance_from_ball_to_segment = distance_from_point_to_segment(ball, segment_start, segment_end);
  if (distance_from_ball_to_segment < 0)
  {
    return false;
  }
  if (distance_from_ball_to_segment < BALL_DIAMETER)
  {
    return true;
  }
  return false;
}

/**
 * Gets the index of the eight ball.
 */
unsigned short int eight_ball_index()
{
  return object_balls.size();
}

/**
 * Gets the obstructing balls lying on the given line segment for a given shot. We use this function to
 * check that no balls are obstructing the path of the cue ball to the object ball and from the object
 * ball to a particular pocket. If a permanent obstruction is found, return obstructions immediately.
 * This function relies on {@code opponent_object_balls}, {@code object_balls}, {@code eight_ball}, and {@code pockets}
 * being populated. If it is the 8 ball shot and we are considering a cue ball path, then treat all obstructions as
 * non-obstructions because they simply block the object ball from scratching.
 *
 * @param object_ball_index the index of the object ball we are shooting at. It is not populated into the list of
 * obstructions since it cannot obstruct itself. The eight ball is represented by an index of eight_ball_index().
 * @param is_cue_ball_path this is a calculation for obstructions to the cue ball path. it determines whether or not
 * to include the pockets on the list of possible obstructions. If we are shooting the 8 ball, we can also ignore
 * all obstructions other than the pockets.
 * TODO: Note which obstructions can be hit softly without consequences.
 */
obstructions_struct get_obstructions_on_segment_for_shot(
    unsigned short int object_ball_index,
    const Vector2d &segment_start,
    const Vector2d &segment_end,
    bool is_cue_ball_path)
{
  obstructions_struct obstructions;
  for (unsigned short int o = 0; o < opponent_object_balls.size(); ++o)
  {
    if (ball_intersects_segment(opponent_object_balls[o], segment_start, segment_end))
    {
      if (is_cue_ball_path && object_ball_index == eight_ball_index())
      {
        obstructions.has_permanent_obstruction = false;
        return obstructions;
      }
      return obstructions;
    }
  }
  if (is_cue_ball_path)
  {
    for (unsigned short int p = 0; p < pockets.size(); ++p)
    {
      if (ball_intersects_segment(pockets[p], segment_start, segment_end))
      {
        return obstructions;
      }
    }
  }
  obstructions.has_permanent_obstruction = false;
  if (object_ball_index == eight_ball_index())
  {
    return obstructions;
  }
  if (ball_intersects_segment(eight_ball, segment_start, segment_end))
  {
    obstructions.has_permanent_obstruction = true;
    return obstructions;
  }
  for (unsigned short int i = 0; i < object_balls.size(); ++i)
  {
    if (i == object_ball_index)
    {
      continue;
    }
    if (ball_intersects_segment(object_balls[i], segment_start, segment_end))
    {
      obstructions.obstructing_object_balls.insert(i);
    }
  }
  return obstructions;
}

/**
 * Populates the ball_to_pocket_obstructions_table. See variable comment.
 * This function relies on {@code opponent_object_balls}, {@code object_balls},
 * {@code eight_ball}, and {@code pockets}
 * being populated.
 */
void populate_ball_to_pocket_obstructions_table()
{
  ball_to_pocket_obstructions_table =
      vector<vector<obstructions_struct>>(object_balls.size() + 1, vector<obstructions_struct>(pockets.size()));
  for (unsigned short int p = 0; p < pockets.size(); ++p)
  {
    Vector2d pocket = pockets[p];
    for (unsigned short int o = 0; o < object_balls.size(); ++o)
    {
      ball_to_pocket_obstructions_table[o][p] =
          get_obstructions_on_segment_for_shot(o, object_balls[o], pocket, false);
    }
    ball_to_pocket_obstructions_table[eight_ball_index()][p] =
        get_obstructions_on_segment_for_shot(eight_ball_index(), eight_ball, pocket, false);
  }
}

/**
 * Calculates the ghost ball position for a given ball shot into a particular pocket.
 * Does not take table edges into account.
 */
ghost_ball_struct get_ghost_ball_for_shot(const Vector2d &ball, const Vector2d &pocket)
{
  ghost_ball_struct ghost_ball;
  Vector2d pocket_to_ball = ball - pocket;
  ghost_ball.coords = ball + pocket_to_ball.normalized() * BALL_DIAMETER;
  if (ghost_ball.coords.x() < 0 || ghost_ball.coords.x() > WIDTH ||
      ghost_ball.coords.y() < 0 || ghost_ball.coords.y() > LENGTH)
  {
    return ghost_ball;
  }
  else
  {
    ghost_ball.possible = true;
  }
  return ghost_ball;
}

/**
 * Populates the ghost ball position table.
 * This function relies on {@code object_balls}, {@code eight_ball}, and {@code pockets} being populated.
 * Does not take table edges into account.
 */
void populate_ghost_ball_position_table()
{
  ghost_ball_position_table =
      vector<vector<ghost_ball_struct>>(object_balls.size() + 1, vector<ghost_ball_struct>(pockets.size()));
  for (unsigned short int p = 0; p < pockets.size(); ++p)
  {
    for (unsigned short int o = 0; o < object_balls.size(); ++o)
    {
      ghost_ball_position_table[o][p] = get_ghost_ball_for_shot(object_balls[o], pockets[p]);
    }
    ghost_ball_position_table[eight_ball_index()][p] = get_ghost_ball_for_shot(eight_ball, pockets[p]);
  }
}

/**
 * Modifies set 1 by adding all the elements from set 2.
 */
void insert_into_set(set<unsigned short int> &set1, set<unsigned short int> &set2)
{
  for (unsigned short int index : set2)
  {
    set1.insert(index);
  }
}

/**
 * Populates the shot info table obstructions.
 * Relies on initialize_pockets and populate_ball_to_pocket_obstructions_table, populate_ghost_ball_table.
 */
void populate_shot_info_table_obstructions()
{
  shot_info_table =
      vector<vector<vector<vector<shot_info_struct>>>>(
          WIDTH + 1, vector<vector<vector<shot_info_struct>>>(
                         LENGTH + 1, vector<vector<shot_info_struct>>(
                                         object_balls.size() + 1, vector<shot_info_struct>(pockets.size()))));
  for (unsigned short int o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned short int p = 0; p < pockets.size(); ++p)
    {
      const ghost_ball_struct *ghost_ball_struct_ptr = &ghost_ball_position_table[o][p];
      const obstructions_struct *obstructions_struct_ptr = &ball_to_pocket_obstructions_table[o][p];
      for (unsigned short int w = 0; w <= WIDTH; ++w)
      {
        for (unsigned short int l = 0; l <= LENGTH; ++l)
        {
          shot_info_struct &shot_info = shot_info_table[w][l][o][p];
          shot_info.ghost_ball = ghost_ball_struct_ptr;
          shot_info.ball_to_pocket_obstructions = obstructions_struct_ptr;
          if (!shot_info.ghost_ball->possible)
          {
            shot_info.possible = false;
            continue;
          }
          shot_info.shot_obstructions = *shot_info.ball_to_pocket_obstructions;
          if (shot_info.shot_obstructions.has_permanent_obstruction)
          {
            shot_info.possible = false;
            continue;
          }
          Vector2d cue_ball = move_ball_in_from_rails(Vector2d(w, l));
          obstructions_struct cue_to_object_obstructions =
              get_obstructions_on_segment_for_shot(o, cue_ball, shot_info.ghost_ball->coords, false);
          if (cue_to_object_obstructions.has_permanent_obstruction)
          {
            shot_info.shot_obstructions.has_permanent_obstruction = true;
            shot_info.possible = false;
            continue;
          }
          insert_into_set(
              shot_info.shot_obstructions.obstructing_object_balls,
              cue_to_object_obstructions.obstructing_object_balls);
        }
      }
    }
  }
}

/**
 * Gets a vector from the center of the pocket towards the table.
 * Expects pockets to be in a particular order. (bottom left, bottom right, left, right, upper left, upper right).
 * TODO: Write unit test.
 */
Vector2d get_vector_from_center_of_pocket(unsigned short int pocket_index)
{
  switch (pocket_index)
  {
  case 0:
    return Vector2d(1, 1);
  case 1:
    return Vector2d(-1, 1);
  case 2:
    return Vector2d(1, 0);
  case 3:
    return Vector2d(-1, 0);
  case 4:
    return Vector2d(1, -1);
  case 5:
    return Vector2d(-1, -1);
  }
  return Vector2d(0, 0);
}

/**
 * Gets the shot difficulty given a cue ball, ghost ball location, and a pocket. It does this by
 * projecting the cue ball onto the ghost ball and pocket line. It then multiplies the projected
 * distance by the distance of the ghost ball to the pocket. Represents the shot difficulty
 * stated here: http://www.sfbilliards.com/articles/1994.pdf.
 * Does not take obstructions into account.
 * TODO: Unit test this.
 * TODO: Break out difficulty scaling factor into its own function.
 * If impossible, it returns inf.
 */
float get_shot_difficulty(const Vector2d &cue_ball, const Vector2d &ghost_ball, unsigned short int pocket_index)
{
  Vector2d vector_from_center_of_pocket = get_vector_from_center_of_pocket(pocket_index);
  if (cue_ball == ghost_ball)
  {
    return 0;
  }
  Vector2d pocket = pockets[pocket_index];
  Vector2d ghost_ball_to_pocket = pocket - ghost_ball;
  Vector2d pocket_to_ghost_ball = -1 * ghost_ball_to_pocket;
  // Equal to the inverse of the cosine of the angle of the shot.
  float difficulty_scaling_factor_due_to_angle =
      (pocket_to_ghost_ball.norm() * vector_from_center_of_pocket.norm()) /
      pocket_to_ghost_ball.dot(vector_from_center_of_pocket);
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  float projected_distance_of_cue_ball_to_shot_line =
      ghost_ball_to_cue_ball.squaredNorm() * ghost_ball_to_pocket.norm() /
      ghost_ball_to_pocket.dot(ghost_ball_to_cue_ball);
  if (projected_distance_of_cue_ball_to_shot_line > 0)
  {
    return std::numeric_limits<float>::infinity();
  }
  return -1 * projected_distance_of_cue_ball_to_shot_line *
         ghost_ball_to_pocket.norm() * difficulty_scaling_factor_due_to_angle;
}

/**
 * Populates the shot info table with the difficulty of each shot. Does not take the next shot into account.
 * Simply calculates the difficulty of making the given shot.
 */
void populate_shot_info_table_difficulty()
{
  for (unsigned short int o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned short int p = 0; p < pockets.size(); ++p)
    {
      const ghost_ball_struct &ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned short int w = 0; w <= WIDTH; ++w)
      {
        for (unsigned short int l = 0; l <= LENGTH; ++l)
        {
          shot_info_struct &shot_info = shot_info_table[w][l][o][p];
          if (!shot_info.possible)
          {
            shot_info.difficulty = std::numeric_limits<float>::infinity();
          }
          else
          {
            shot_info.difficulty = get_shot_difficulty(move_ball_in_from_rails(Vector2d(w, l)), ghost_ball_position.coords, p);
            if (shot_info.difficulty > MAX_DIFFICULTY_SHOT_TO_CONSIDER)
            {
              shot_info.possible = false;
            }
          }
        }
      }
    }
  }
}

/**
 * Returns whether two doubles are equal to each other.
 */
bool double_equals(double a, double b, double epsilon = 0.001)
{
  return std::abs(a - b) < epsilon;
}

/**
 * Gets the intersection coordinate of two line segments.
 * Uses the method outlined here:
 * https://www.topcoder.com/community/competitive-programming/tutorials/geometry-concepts-line-intersection-and-its-applications/
 */
segment_intersection_struct get_intersection_of_line_segments(
    const Vector2d &segment1_start, const Vector2d &segment1_end,
    const Vector2d &segment2_start, const Vector2d &segment2_end)
{
  segment_range_struct segment1_ranges = get_segment_ranges(segment1_start, segment1_end);
  segment_range_struct segment2_ranges = get_segment_ranges(segment2_start, segment2_end);
  float A1 = segment1_end.y() - segment1_start.y();
  float B1 = segment1_start.x() - segment1_end.x();
  float C1 = A1 * segment1_start.x() + B1 * segment1_start.y();
  float A2 = segment2_end.y() - segment2_start.y();
  float B2 = segment2_start.x() - segment2_end.x();
  float C2 = A2 * segment2_start.x() + B2 * segment2_start.y();
  double det = A1 * B2 - A2 * B1;
  segment_intersection_struct segment_intersection;
  if (double_equals(det, 0))
  {
    return segment_intersection;
  }
  double x = (B2 * C1 - B1 * C2) / det;
  double y = (A1 * C2 - A2 * C1) / det;
  if (
      !double_equals(x, segment1_ranges.min_x) &&
      !double_equals(x, segment1_ranges.max_x) &&
      (x < segment1_ranges.min_x || x > segment1_ranges.max_x))
  {
    return segment_intersection;
  }
  else if (
      !double_equals(y, segment1_ranges.min_y) &&
      !double_equals(y, segment1_ranges.max_y) &&
      (y < segment1_ranges.min_y || y > segment1_ranges.max_y))
  {
    return segment_intersection;
  }
  else if (
      !double_equals(x, segment2_ranges.min_x) &&
      !double_equals(x, segment2_ranges.max_x) &&
      (x < segment2_ranges.min_x || x > segment2_ranges.max_x))
  {
    return segment_intersection;
  }
  else if (
      !double_equals(y, segment2_ranges.min_y) &&
      !double_equals(y, segment2_ranges.max_y) &&
      (y < segment2_ranges.min_y || y > segment2_ranges.max_y))
  {
    return segment_intersection;
  }
  segment_intersection.has_intersection = true;
  segment_intersection.intersection_point = Vector2d(x, y);
  return segment_intersection;
}

/**
 * Gets the intersection coordinate of two line segments. This is a convenience method used to take in a two element
 * vector used to represent pockets.
 */
segment_intersection_struct get_intersection_of_line_segments(
    const Vector2d &segment1_start, const Vector2d &segment1_end,
    const vector<Vector2d> &segment2)
{
  return get_intersection_of_line_segments(segment1_start, segment1_end, segment2[0], segment2[1]);
}

/**
 * Gets the path of a ball after it bounces off table edges.
 * Depends on pocket and table edges being initialized.
 * segment_start is assumed to be within the table.
 * segment_length is passed in for optimization purposes. It can also be calculated using segment_start
 * and segment_end.
 */
single_rail_reflection_struct reflect_ball_path_off_single_table_edge(
    const Vector2d &segment_start, const Vector2d &segment_end, float segment_length)
{
  single_rail_reflection_struct rail_reflection;
  rail_reflection.has_reflection = false;
  Vector2d direction_of_final_vector;

  if (segment_end.x() < left_edge[0].x())
  {
    segment_intersection_struct segment_intersection =
        get_intersection_of_line_segments(segment_start, segment_end, left_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(-1 * start_to_end.x(), start_to_end.y()).normalized();
      rail_reflection.has_reflection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (segment_end.x() > right_edge[0].x())
  {
    segment_intersection_struct segment_intersection =
        get_intersection_of_line_segments(segment_start, segment_end, right_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(-1 * start_to_end.x(), start_to_end.y()).normalized();
      rail_reflection.has_reflection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (segment_end.y() < bottom_edge[0].y())
  {
    segment_intersection_struct segment_intersection =
        get_intersection_of_line_segments(segment_start, segment_end, bottom_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(start_to_end.x(), -1 * start_to_end.y()).normalized();
      rail_reflection.has_reflection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (segment_end.y() > top_edge[0].y())
  {
    segment_intersection_struct segment_intersection =
        get_intersection_of_line_segments(segment_start, segment_end, top_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(start_to_end.x(), -1 * start_to_end.y()).normalized();
      rail_reflection.has_reflection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (!rail_reflection.has_reflection)
  {
    return rail_reflection;
  }

  float start_to_rail_length = (rail_reflection.intersection_point - segment_start).norm();
  float rail_to_end_length = segment_length - start_to_rail_length;
  rail_reflection.end_point = rail_reflection.intersection_point + rail_to_end_length * direction_of_final_vector;
  rail_reflection.distance_traveled_after_rail = rail_to_end_length;
  return rail_reflection;
}

/**
 * Converts the strength given to the distance traveled by the cue ball if it does not
 * contact an object ball.
 * TODO: Expects 12 strengths.
 * TODO: Scale the strength to the distance non linearly.
 * TODO: Unit test.
 */
float strength_to_distance(unsigned short int strength)
{
  float multiplier = 1;
  switch (strength)
  {
  case 0:
    return 0;
  case 1:
    multiplier = 0.033;
    break;
  case 2:
    multiplier = 0.067;
    break;
  case 3:
    multiplier = 0.1;
    break;
  case 4:
    multiplier = 0.133;
    break;
  case 5:
    multiplier = 0.167;
    break;
  case 6:
    multiplier = 0.2;
    break;
  case 7:
    multiplier = 0.267;
    break;
  case 8:
    multiplier = 0.4;
    break;
  case 9:
    multiplier = 0.533;
    break;
  case 10:
    multiplier = 0.733;
    break;
  case 11:
    multiplier = 1;
    break;
  }
  return multiplier * MAX_DIAMONDS * UNITS_PER_DIAMOND;
}

/**
 * Gets the unit tangent line vector for a given cue ball, ghost ball, and pocket and other shot angle info.
 * To calculate minimum travel distance of the cue ball, the following paper is used:
 *   https://billiards.colostate.edu/technical_proofs/TP_3-2.pdf
 */
shot_angle_struct get_shot_angle(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Vector2d &pocket)
{
  shot_angle_struct shot_angle;
  shot_angle.origin = ghost_ball;
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  Vector2d ghost_ball_to_pocket = pocket - ghost_ball;
  shot_angle.pocket_direction = ghost_ball_to_pocket.normalized();
  float determinant = ghost_ball_to_cue_ball.x() *
                          ghost_ball_to_pocket.y() -
                      ghost_ball_to_cue_ball.y() * ghost_ball_to_pocket.x();
  Vector2d tangent_line_direction = ghost_ball_to_pocket.unitOrthogonal();
  if (determinant < 0)
  {
    shot_angle.follow_direction = shot_angle_struct::COUNTER_CLOCKWISE;
    shot_angle.tangent_line_direction = Vector2d(-1 * tangent_line_direction.x(), -1 * tangent_line_direction.y());
  }
  else
  {
    shot_angle.follow_direction = shot_angle_struct::CLOCKWISE;
    shot_angle.tangent_line_direction = tangent_line_direction;
  }
  double dot = ghost_ball_to_cue_ball.dot(ghost_ball_to_pocket);
  float negative_cos_theta = dot / (ghost_ball_to_cue_ball.norm() * ghost_ball_to_pocket.norm());
  shot_angle.cut_angle_in_radians = acos(-1 * negative_cos_theta);
  float cos_squared = negative_cos_theta * negative_cos_theta;
  float sin_squared = 1 - cos_squared;
  shot_angle.fractional_distance = sin_squared;
  shot_angle.minimum_travel_distance_of_cue_ball = ghost_ball_to_pocket.norm() * sin_squared / cos_squared;
  shot_angle.cue_ball_to_ghost_ball = -1 * ghost_ball_to_cue_ball;
  return shot_angle;
}

// In radians. Uses estimates from https://billiards.colostate.edu/technical_proofs/new/TP_B-14.pdf.
// This is for 'good' draw.
// This is how much it's deflected from the original aim line (ghost ball to cue ball.)
float estimate_draw_angle_from_cut_angle(float cut_angle) {
  if (cut_angle < 0.349) {
    return cut_angle * 4;
  } else if (cut_angle < 40) {
    return cut_angle * 3;
  } else {
    return 4 * cut_angle / 3 + 1.047;
  }
}

// In radians. Uses estimates from https://billiards.colostate.edu/technical_proofs/new/TP_B-13.pdf.
// This is for 'good' follow.
// This is how much it is deflected from the original aim line.
float estimate_follow_angle_from_cut_angle(float cut_angle) {
  if (cut_angle < 0.253) {
    return cut_angle * 3;
  } else if (cut_angle < 0.848) {
    return cut_angle * 0.611;
  } else {
    float ret = 1.1 - cut_angle;
    if (ret < 0) {
      return 0;
    }
    return ret;
  }
}

/**
 * Gets the path given the shot angle info, a strength, and a spin.
 * Relies on pockets and table edges being populated.
 * TODO: Break this up into multiple functions.
 * TODO: This is just a hack right now. Does not take spin into account.
 */
vector<Vector2d> get_path(shot_angle_struct shot_angle, unsigned short int strength, unsigned short int spin)
{
  vector<Vector2d> ret;
  float distance_to_travel = shot_angle.fractional_distance * strength_to_distance(strength);
  Vector2d cue_ball_destination = shot_angle.tangent_line_direction * distance_to_travel + shot_angle.origin;
  Vector2d cue_ball_location = shot_angle.origin;
  ret.push_back(cue_ball_location);
  single_rail_reflection_struct rail_reflection = reflect_ball_path_off_single_table_edge(
      cue_ball_location, cue_ball_destination, distance_to_travel);
  while (rail_reflection.has_reflection)
  {
    if (!double_equals(rail_reflection.intersection_point.x(), cue_ball_location.x()) ||
        !double_equals(rail_reflection.intersection_point.y(), cue_ball_location.y()))
    {
      ret.push_back(rail_reflection.intersection_point);
    }
    cue_ball_location = rail_reflection.intersection_point;
    cue_ball_destination = rail_reflection.end_point;
    distance_to_travel = rail_reflection.distance_traveled_after_rail;
    rail_reflection = reflect_ball_path_off_single_table_edge(
        cue_ball_location, cue_ball_destination, distance_to_travel);
  }
  ret.push_back(cue_ball_destination);

  return ret;
}

/**
 * Populates the table of shot paths.
 * TODO: Take the spins into account when calculating the shot paths.
 * TODO: Possible optimization is to populate a map of map[object balls][pockets][angles][num strength][num spins]
 * and use that to populate the full table of map[object balls][pockets][cue ball positions][num strengths][num spins].
 * TODO: Break up this function.
 */
void populate_shot_path_table()
{
  shot_path_table =
      vector<vector<vector<vector<vector<vector<shot_path_struct>>>>>>(
          WIDTH + 1, vector<vector<vector<vector<vector<shot_path_struct>>>>>(
                         LENGTH + 1, vector<vector<vector<vector<shot_path_struct>>>>(
                                         object_balls.size() + 1, vector<vector<vector<shot_path_struct>>>(
                                                                      pockets.size(), vector<vector<shot_path_struct>>(
                                                                                          NUM_STRENGTHS, vector<shot_path_struct>(
                                                                                                             NUM_SPINS))))));
  for (unsigned short int o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned short int p = 0; p < pockets.size(); ++p)
    {
      for (unsigned short int w = 0; w <= WIDTH; ++w)
      {
        for (unsigned short int l = 0; l <= LENGTH; ++l)
        {
          const ghost_ball_struct &ghost_ball_position = *shot_info_table[w][l][o][p].ghost_ball;
          shot_angle_struct shot_angle = get_shot_angle(move_ball_in_from_rails(Vector2d(w, l)), ghost_ball_position.coords, pockets[p]);
          for (unsigned short int st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned short int sp = 0; sp < NUM_SPINS; ++sp)
            {
              shot_path_struct &current_shot_path = shot_path_table[w][l][o][p][st][sp];
              if (!shot_info_table[w][l][o][p].possible)
              {
                continue;
              }
              float distance_to_travel = shot_angle.fractional_distance * strength_to_distance(st);
              if (distance_to_travel < shot_angle.minimum_travel_distance_of_cue_ball) {
                continue;
              }
              current_shot_path.possible = true;
              vector<Vector2d> path = get_path(shot_angle, st, sp);
              current_shot_path.path_segments = path;
              current_shot_path.final_position = path[path.size() - 1];
              for (unsigned short int pa = 0; pa < path.size() - 1; ++pa)
              {
                obstructions_struct obstructions_on_segment =
                    get_obstructions_on_segment_for_shot(o, path[pa], path[pa + 1], true);
                if (obstructions_on_segment.has_permanent_obstruction)
                {
                  current_shot_path.shot_obstructions.has_permanent_obstruction = true;
                  current_shot_path.possible = false;
                  break;
                }
                current_shot_path.shot_obstructions.has_permanent_obstruction = false;
                current_shot_path.possible = true;
                insert_into_set(
                    current_shot_path.shot_obstructions.obstructing_object_balls,
                    obstructions_on_segment.obstructing_object_balls);
              }
            }
          }
        }
      }
    }
  }
}

/**
 * Populates the table of shot info and paths.
 */
void populate_shot_info_and_path_table()
{
  shot_info_and_path_table =
      vector<vector<vector<vector<vector<vector<shot_info_and_path_struct>>>>>>(
          WIDTH + 1, vector<vector<vector<vector<vector<shot_info_and_path_struct>>>>>(
                         LENGTH + 1, vector<vector<vector<vector<shot_info_and_path_struct>>>>(
                                         object_balls.size() + 1, vector<vector<vector<shot_info_and_path_struct>>>(
                                                                      pockets.size(), vector<vector<shot_info_and_path_struct>>(
                                                                                          NUM_STRENGTHS, vector<shot_info_and_path_struct>(
                                                                                                             NUM_SPINS))))));
  for (unsigned short int o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned short int p = 0; p < pockets.size(); ++p)
    {
      for (unsigned short int w = 0; w <= WIDTH; ++w)
      {
        for (unsigned short int l = 0; l <= LENGTH; ++l)
        {
          const shot_info_struct *shot_info = &shot_info_table[w][l][o][p];
          for (unsigned short int st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned short int sp = 0; sp < NUM_SPINS; ++sp)
            {
              shot_info_and_path_struct &current_shot_info_and_path = shot_info_and_path_table[w][l][o][p][st][sp];
              current_shot_info_and_path.shot_path = &shot_path_table[w][l][o][p][st][sp];
              current_shot_info_and_path.shot_info = shot_info;
              if (!current_shot_info_and_path.shot_info->possible || !current_shot_info_and_path.shot_path->possible)
              {
                current_shot_info_and_path.possible = false;
                continue;
              }
              current_shot_info_and_path.difficulty =
                shot_info->difficulty * (1 + (strength_to_distance(st) / MAX_DIAMONDS * UNITS_PER_DIAMOND * 0.33));
              current_shot_info_and_path.weighted_difficulty =
                current_shot_info_and_path.difficulty * current_shot_info_and_path.difficulty;
            }
          }
        }
      }
    }
  }
}

void populate_eight_ball_in_selected_shot_table()
{
  for (unsigned short int w = 0; w <= WIDTH; ++w)
  {
    for (unsigned short int l = 0; l <= LENGTH; ++l)
    {
      selected_shot_struct &selected_shot = selected_shot_table[w][l][0];
      for (unsigned short int p = 0; p < pockets.size(); ++p)
      {
        shot_info_struct &current_shot_info = shot_info_table[w][l][eight_ball_index()][p];
        if (current_shot_info.shot_obstructions.has_permanent_obstruction ||
            current_shot_info.difficulty > MAX_DIFFICULTY_SHOT_TO_CONSIDER)
        {
          continue;
        }
        for (unsigned short int st = 0; st < NUM_STRENGTHS; ++st)
        {
          for (unsigned short int sp = 0; sp < NUM_SPINS; ++sp)
          {
            shot_info_and_path_struct &current_shot_info_and_path = shot_info_and_path_table[w][l][eight_ball_index()][p][st][sp];
            const shot_path_struct &current_shot_path = *current_shot_info_and_path.shot_path;
            if (current_shot_info_and_path.possible &&
                current_shot_info_and_path.weighted_difficulty < selected_shot.total_weighted_difficulty)
            {
              selected_shot.possible = true;
              selected_shot.strength = st;
              selected_shot.spin = sp;
              selected_shot.object_ball = eight_ball_index();
              selected_shot.pocket = p;
              selected_shot.total_weighted_difficulty = current_shot_info_and_path.weighted_difficulty;
              selected_shot.current_weighted_difficulty = current_shot_info_and_path.weighted_difficulty;
              selected_shot.path_segments = current_shot_path.path_segments;
            }
          }
        }
      }
    }
  }
}

/**
 * Gets the difficulty of the rest of the shots in the runout if the selected shot is chosen.
 */
remaining_runout_struct get_remaining_runout_difficulty(const shot_info_and_path_struct& current_shot_info_and_path, const shot_path_struct& current_shot_path, unsigned short int combo, set<unsigned short int> &balls, unsigned short int ball) {
  remaining_runout_struct ret;
  if (!current_shot_info_and_path.possible)
  {
    ret.difficulty = std::numeric_limits<float>::infinity();
    return ret;
  }
  set<unsigned short int> intersect;
  set_intersection(
      balls.begin(),
      balls.end(),
      current_shot_path.shot_obstructions.obstructing_object_balls.begin(),
      current_shot_path.shot_obstructions.obstructing_object_balls.end(),
      std::inserter(intersect, intersect.begin()));
  if (intersect.size() > 0)
  {
    ret.difficulty = std::numeric_limits<float>::infinity();
    return ret;
  }
  ret.cue_ball_location_considered = current_shot_path.final_position;
  int next_combo = combo - (1 << ball);
  unsigned short int rounded_x = (unsigned short int)(0.5 + current_shot_path.final_position.x());
  unsigned short int rounded_y = (unsigned short int)(0.5 + current_shot_path.final_position.y());
  if (!selected_shot_table[rounded_x][rounded_y][next_combo].possible)
  {
    ret.difficulty = std::numeric_limits<float>::infinity();
    return ret;
  }
  ret.next_object_ball = selected_shot_table[rounded_x][rounded_y][next_combo].object_ball;
  ret.difficulty = selected_shot_table[rounded_x][rounded_y][next_combo].total_weighted_difficulty;
  return ret;
}

/**
 * This assumes that position is made 50% of the time, 25% it's shot too softly and 25% of the time it's shot too hard.
 * In the case of the misses, the strength unit is increased by one.
 */
void populate_single_combination_in_selected_shot_table(int combo, set<unsigned short int> &balls)
{
  for (auto ball : balls)
  {
    for (unsigned short int w = 0; w <= WIDTH; ++w)
    {
      for (unsigned short int l = 0; l <= LENGTH; ++l)
      {
        selected_shot_struct &selected_shot = selected_shot_table[w][l][combo];
        for (unsigned short int p = 0; p < pockets.size(); ++p)
        {
          shot_info_struct &current_shot_info = shot_info_table[w][l][ball][p];
          if (current_shot_info.shot_obstructions.has_permanent_obstruction ||
              current_shot_info.difficulty > MAX_DIFFICULTY_SHOT_TO_CONSIDER)
          {
            continue;
          }
          set<unsigned short int> intersect_cue_ball_path;
          set_intersection(
              balls.begin(),
              balls.end(),
              current_shot_info.shot_obstructions.obstructing_object_balls.begin(),
              current_shot_info.shot_obstructions.obstructing_object_balls.end(),
              std::inserter(intersect_cue_ball_path, intersect_cue_ball_path.begin()));
          if (intersect_cue_ball_path.size() > 0)
          {
            continue;
          }
          for (unsigned short int st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned short int sp = 0; sp < NUM_SPINS; ++sp)
            {
              unsigned short int missed_strength_1;
              if (st == 0) {
                missed_strength_1 = 1;
              } else {
                missed_strength_1 = st - 1;
              }
              unsigned short int missed_strength_2 = st + 1;
              if (missed_strength_2 >= NUM_STRENGTHS) {
                missed_strength_2 = st - 2;
              }
              const shot_info_and_path_struct & current_shot_info_and_path = shot_info_and_path_table[w][l][ball][p][st][sp];
              shot_path_struct &current_shot_path = shot_path_table[w][l][ball][p][st][sp];
              shot_path_struct &missed_shot_path_1 = shot_path_table[w][l][ball][p][missed_strength_1][sp];
              shot_path_struct &missed_shot_path_2 = shot_path_table[w][l][ball][p][missed_strength_2][sp];
              remaining_runout_struct expected_shot_remaining_runout =
                  get_remaining_runout_difficulty(current_shot_info_and_path, current_shot_path, combo, balls, ball);
              remaining_runout_struct missed_shot_1_remaining_runout =
                  get_remaining_runout_difficulty(current_shot_info_and_path, missed_shot_path_1, combo, balls, ball);
              remaining_runout_struct missed_shot_2_remaining_runout =
                  get_remaining_runout_difficulty(current_shot_info_and_path, missed_shot_path_2, combo, balls, ball);
              if (expected_shot_remaining_runout.difficulty == std::numeric_limits<float>::infinity()) {
                continue;
              }
              if (missed_shot_1_remaining_runout.difficulty == std::numeric_limits<float>::infinity()) {
                continue;
              }
              if (missed_shot_2_remaining_runout.difficulty == std::numeric_limits<float>::infinity()) {
                continue;
              }
              int next_combo = combo - (1 << ball);
              unsigned short int rounded_x = (unsigned short int)(0.5 + current_shot_path.final_position.x());
              unsigned short int rounded_y = (unsigned short int)(0.5 + current_shot_path.final_position.y());
              float new_weighted_difficulty =
                  current_shot_info_and_path.weighted_difficulty +
                  0.5 * expected_shot_remaining_runout.difficulty +
                  0.25 * missed_shot_1_remaining_runout.difficulty +
                  0.25 * missed_shot_2_remaining_runout.difficulty;
              if (new_weighted_difficulty < selected_shot.total_weighted_difficulty)
              {
                selected_shot.next_combo = next_combo;
                selected_shot.possible = true;
                selected_shot.strength = st;
                selected_shot.spin = sp;
                selected_shot.object_ball = ball;
                selected_shot.pocket = p;
                selected_shot.current_weighted_difficulty = current_shot_info_and_path.weighted_difficulty;
                selected_shot.total_weighted_difficulty = new_weighted_difficulty;
                selected_shot.path_segments = current_shot_path.path_segments;
                selected_shot.next_x = rounded_x;
                selected_shot.next_y = rounded_y;
                selected_shot.expected_cue_ball = expected_shot_remaining_runout.cue_ball_location_considered;
                selected_shot.missed_cue_ball_1 = missed_shot_1_remaining_runout.cue_ball_location_considered;
                selected_shot.missed_cue_ball_2 = missed_shot_2_remaining_runout.cue_ball_location_considered;
                selected_shot.missed_1_object_ball = missed_shot_1_remaining_runout.next_object_ball;
                selected_shot.missed_2_object_ball = missed_shot_2_remaining_runout.next_object_ball;
                selected_shot.expected_object_ball = expected_shot_remaining_runout.next_object_ball;
              }
            }
          }
        }
      }
    }
  }
}

/**
 * Process a given combination of object balls.
 * combo represents the indices of object balls to consider. It is represented as a number.
 */
void process_object_ball_combination(int num_object_balls, int combo)
{
  set<unsigned short int> balls;
  for (int i = 0; i < num_object_balls; ++i)
  {
    if ((combo >> i) & 1)
      balls.insert(i);
  }
  populate_single_combination_in_selected_shot_table(combo, balls);
}

/**
 * Gets all possible combinations of k elements from the range (1..c) inclusive.
 */
void process_object_ball_combinations(int num_object_balls, int num_elements)
{
  int n = num_object_balls;
  int combo = (1 << num_elements) - 1; // k bit sets
  while (combo < 1 << n)
  {
    process_object_ball_combination(num_object_balls, combo);

    int x = combo & -combo;
    int y = combo + x;
    int z = (combo & ~y);
    combo = z / x;
    combo >>= 1;
    combo |= y;
  }
}

/**
 * Populates the selected shot table.
 */
void populate_selected_shot_table()
{
  selected_shot_table =
      vector<vector<vector<selected_shot_struct>>>(
          WIDTH + 1, vector<vector<selected_shot_struct>>(
                         LENGTH + 1, vector<selected_shot_struct>((int)pow(2, object_balls.size()))));
  populate_eight_ball_in_selected_shot_table();
  for (unsigned short int i = 1; i <= object_balls.size(); ++i)
  {
    process_object_ball_combinations(object_balls.size(), i);
  }
}

const Vector2d find_ball_in_hand_solution()
{
  unsigned short int max_object_ball_index = (unsigned short int)pow(2, object_balls.size()) - 1;
  float x = 0;
  float y = 0;
  float minimum_difficulty = std::numeric_limits<float>::infinity();
  for (unsigned short int w = 0; w <= WIDTH; ++w)
  {
    for (unsigned short int l = 0; l <= LENGTH; ++l)
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

string zero_based_index_to_one_based_index_string(unsigned short int index)
{
  return to_string(index + 1);
}

string object_ball_index_to_string(unsigned short int index)
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

string pocket_to_string(unsigned short int pocket_index)
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
string spin_to_string(unsigned short int spin_index)
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

string strength_to_string(unsigned short int strength)
{
  return to_string(strength + 1) +
         " - " +
         to_string((int)round(strength_to_distance(strength) / (MAX_DIAMONDS * UNITS_PER_DIAMOND) * 100)) + "%";
}

string unsigned_short_int_coordinates_struct_to_string(unsigned_short_int_coordinates_struct coordinates)
{
  return vector_to_string(Vector2d(coordinates.x, coordinates.y));
}

void display_solution(Vector2d intiial_cue_ball)
{
  unsigned short int combo = (unsigned short int)pow(2, object_balls.size()) - 1;

  unsigned_short_int_coordinates_struct coords;
  coords.x = (unsigned short int) (intiial_cue_ball.x() + 0.5);
  coords.y = (unsigned short int) (intiial_cue_ball.y() + 0.5);
  unsigned short int shot_number = 1;
  cout << endl;
  cout << "Table layout: " << endl;
  cout << "-------------" << endl;
  cout << "Ball in hand: " << (ball_in_hand ? "True" : "False") << endl;
  cout << "Cue ball: " << unsigned_short_int_coordinates_struct_to_string(coords) << endl;
  cout << "Eight ball: " << vector_to_string(eight_ball) << endl;
  cout << "Object balls: ";
  for (unsigned short int o = 0; o < object_balls.size(); ++o)
  {
    cout << vector_to_string(object_balls[o]) << " ";
  }
  cout << endl;
  cout << "Opponent balls: ";
  for (unsigned short int o = 0; o < opponent_object_balls.size(); ++o)
  {
    cout << vector_to_string(opponent_object_balls[o]) << " ";
  }
  cout << endl;
  do
  {
    selected_shot_struct selected_shot = selected_shot_table[coords.x][coords.y][combo];
    if (!selected_shot.possible)
    {
      cout << "No solution." << endl;
      return;
    }
    cout << "\nShot " << (int)shot_number << ":\n";
    cout << "-------\n";
    cout << "Cue ball position: " + unsigned_short_int_coordinates_struct_to_string(coords) + "\n";
    cout << "Object ball target: " << object_ball_index_to_string(selected_shot.object_ball) << endl;
    cout << "Pocket: " << pocket_to_string(selected_shot.pocket) << endl;
    cout << "Difficulty from this shot onwards: " << selected_shot.total_weighted_difficulty << endl;
    cout << "Difficulty of this shot: " << selected_shot.current_weighted_difficulty << endl;
    cout << "Strength (1-" << to_string(NUM_STRENGTHS) << "): " << strength_to_string(selected_shot.strength) << endl;
    cout << "Spin: " << spin_to_string(selected_shot.spin) << endl;
    cout << "Cue ball path: " << path_to_string(selected_shot.path_segments) << endl;
    if (combo == 0)
    {
      break;
    }
    coords.x = selected_shot.next_x;
    coords.y = selected_shot.next_y;
    combo = selected_shot.next_combo;
    shot_number += 1;
  } while (true);
  cout << "Done" << endl;
}

void write_to_file(string json)
{
  ofstream myfile;
  myfile.open(GAME_DATA_FILE);
  myfile << "game_data = " << json << "\n";
  myfile.close();
}

string get_json_for_solution(Vector2d initial_cue_ball)
{
  json game_data;

  unsigned short int combo = (unsigned short int)pow(2, object_balls.size()) - 1;

  unsigned_short_int_coordinates_struct coords;
  coords.x = (unsigned short int) (initial_cue_ball.x() + 0.5);
  coords.y = (unsigned short int) (initial_cue_ball.y() + 0.5);
  unsigned short int shot_number = 1;

  game_data["units_per_diamond"] = UNITS_PER_DIAMOND;
  game_data["ball_in_hand"] = ball_in_hand;
  game_data["cue_ball"] = {initial_cue_ball.x(), initial_cue_ball.y()};
  game_data["eight_ball"] = {eight_ball.x(), eight_ball.y()};
  game_data["object_balls"] = {};
  game_data["turns"] = {};
  game_data["has_solution"] = false;
  for (unsigned short int o = 0; o < object_balls.size(); ++o)
  {
    game_data["object_balls"].push_back({object_balls[o].x(), object_balls[o].y()});
  }
  game_data["object_balls"].push_back(game_data["eight_ball"]);
  game_data["opponent_object_balls"] = {};
  for (unsigned short int o = 0; o < opponent_object_balls.size(); ++o)
  {
    game_data["opponent_object_balls"].push_back({opponent_object_balls[o].x(), opponent_object_balls[o].y()});
  }
  do
  {
    selected_shot_struct selected_shot = selected_shot_table[coords.x][coords.y][combo];

    if (!selected_shot.possible)
    {
      return game_data.dump();
    }
    game_data["has_solution"] = true;
    json turn;
    turn["shot_number"] = shot_number;
    Vector2d fixed_cue_ball_position = move_ball_in_from_rails(Vector2d(coords.x, coords.y));
    turn["cue_ball"] = {fixed_cue_ball_position.x(), fixed_cue_ball_position.y()};
    turn["object_ball_index"] = selected_shot.object_ball;
    turn["pocket_index"] = selected_shot.pocket;
    turn["pocket_coords"] = {pockets[selected_shot.pocket].x(), pockets[selected_shot.pocket].y()};
    turn["runout_difficulty"] = selected_shot.total_weighted_difficulty;
    turn["shot_difficulty"] = selected_shot.current_weighted_difficulty;
    turn["strength"] = selected_shot.strength;
    turn["spin"] = selected_shot.spin;
    turn["expected_cue_ball"] = {selected_shot.expected_cue_ball.x(), selected_shot.expected_cue_ball.y()};
    turn["missed_cue_ball_1"] = {selected_shot.missed_cue_ball_1.x(), selected_shot.missed_cue_ball_1.y()};
    turn["missed_cue_ball_2"] = {selected_shot.missed_cue_ball_2.x(), selected_shot.missed_cue_ball_2.y()};
    turn["missed_1_object_ball_index"] = selected_shot.missed_1_object_ball;
    turn["missed_2_object_ball_index"] = selected_shot.missed_2_object_ball;
    turn["expected_object_ball_index"] = selected_shot.expected_object_ball;

    json path;
    for (unsigned short int i = 0; i < selected_shot.path_segments.size(); ++i)
    {
      path.push_back({selected_shot.path_segments[i].x(), selected_shot.path_segments[i].y()});
    }
    turn["path"] = path;
    game_data["turns"].push_back(turn);
    if (combo == 0)
    {
      break;
    }
    coords.x = selected_shot.next_x;
    coords.y = selected_shot.next_y;
    combo = selected_shot.next_combo;
    shot_number += 1;
  } while (true);
  return game_data.dump();
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
