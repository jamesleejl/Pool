#include <iostream>
#include <vector>
#include <set>
#include <chrono>
#include <ctime>

#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

// TODO: Handle bank shots.
// TODO: Handle side pocket shots correctly. https://billiards.colostate.edu/threads/pocket.html
// TODO: Handle minimum distance cue ball must travel to sink an object ball for scratch purposes and other reasons.

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
   * Only populated if has_permanent_obstruction is false.
   */
  set<unsigned char> obstructing_object_balls;
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
   * If there is an intersection point, these are the coordiantes.
   */
  Vector2d intersection_point;
};

/**
 * Holds information about a ball path after reflection off a rail of a pool table.
 */
struct rail_reflection_struct
{
  /**
   * Whether or not there is an intersection between the line segments.
   */
  bool has_intersection = false;
  /**
   * If there is an intersection point, these are the coordiantes.
   */
  Vector2d intersection_point;
  /**
   * After reflection off the rail, this is the new end point of the cue ball.
   */ 
  Vector2d end_point;
  /**
   * The distance the ball traveled along this path after hitting the rail to get to the end point.
   */
  float distance_traveled_after_rail;
};

/**
 * Holds information about the minimum and maximum values of line segments.
 */
struct segment_range_struct
{
  /**
   * The minimum x coordinate of the segments given.
   */
  float min_x;
  /**
   * The maximum x coordinate of the segments given.
   */
  float max_x;
  /**
   * The minimum y coordinate of the segments given.
   */
  float min_y;
  /**
   * The maximum y coordinate of the segments given.
   */
  float max_y;
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
   */
  unsigned char strength;
  /**
   * The spin of the shot. (0..NUM_SPINS)
   */
  unsigned char spin;
  /**
   * The index of the object ball to shoot. The eight ball is at object_balls.size().
   */
  unsigned char object_ball;
  /**
   * Which pocket index to shoot the ball into.
   */
  unsigned char pocket;
  /**
   * The total weighted difficulty of the runout starting from this ball.
   */
  float total_weighted_difficulty = std::numeric_limits<float>::infinity();
  /**
   * The points defining the line segments the cue ball travels along after contact with the object ball.
   */
  vector<Vector2d> path_segments;
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
};

/**
 * The information about the cue ball path after a shot.
 */
struct shot_path_struct
{
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
   * Whether the shot path is possible or not. This is based on whether the shot is too difficult or not.
   * If this is false, the shot path is not populated.
   */
  bool within_acceptable_difficulty = false;
};

/**
 * The possible information with regards to a shot.
 */
struct shot_info_struct
{
  /**
   * The difficulty of the shot.
   */
  float difficulty = std::numeric_limits<float>::infinity();
  /**
   * The weighted difficulty of the shot so that we can just add numbers together when considering runout paths.
   * TODO: Make this weighting more accurate.
   */
  float weighted_difficulty = std::numeric_limits<float>::infinity();
  /**
   * The obstructions for this shot.
   */
  obstructions_struct shot_obstructions;
};

/**
 * The length of a diamond in units.
 */
const unsigned char UNITS_PER_DIAMOND = 2;
/**
 * The number of shot strengths to consider
 */
const unsigned char NUM_STRENGTHS = 12;
/**
 * The number of shot spins to consider.
 */
const unsigned char NUM_SPINS = 5;
/**
 * The maximum difficulty of individual shot to consider shooting.
 */
const unsigned int MAX_DIFFICULTY_SHOT_TO_CONSIDER = 500;
/**
 * The unit diameter of the balls.
 */
const float BALL_DIAMETER = 0.18 * UNITS_PER_DIAMOND;
/**
 * The width of the pool table in units.
 */
const unsigned char WIDTH = UNITS_PER_DIAMOND * 4;
/**
 * The length of the pool table in units.
 */
const unsigned char LENGTH = WIDTH * 2;
/**
 * The maximum number of diamonds the cue ball can travel if untouched by an object ball.
 */
const unsigned char MAX_DIAMONDS = 30;
/**
 * The positions of the pockets.
 */
vector<Vector2d> pockets;
/**
 * The position of the eight ball.
 */
Vector2d eight_ball;
/**
 * The initial position of the cue ball.
 */
Vector2d cue_ball;
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
 * Dimensions are [Object balls Size + 1][Pockets]
 */
vector<vector<obstructions_struct>> ball_to_pocket_obstructions_table;

/**
 * A table of ghost ball positions for a given object ball index to a pocket. Includes the 8 ball at index
 * object_balls.size().
 * Dimensions are [Object balls Size + 1][Pockets]
 */
vector<vector<Vector2d>> ghost_ball_position_table;

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
 * Gets the best shot path to for each given cue ball position for the given layout.
 * Dimensions are [Width][Length][Num table layouts]. The table layout index is generated
 * by summing the (2^object ball indices) of the remaining balls. That is, if there are
 * 3 object balls, their indices will be 1,2,4. A table which consists of balls 1 and
 * 3 will be in position 5. The table with only the 8 ball remaining is at position 0.
 * Dimensions are [Width + 1][Length + 1][Num table layouts].
 */
vector<vector<vector<selected_shot_struct>>> selected_shot_table;

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
  bottom_edge.push_back(pockets[0]);
  bottom_edge.push_back(pockets[1]);
  right_edge.push_back(pockets[1]);
  right_edge.push_back(pockets[5]);
  top_edge.push_back(pockets[5]);
  top_edge.push_back(pockets[4]);
  left_edge.push_back(pockets[4]);
  left_edge.push_back(pockets[0]);
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
unsigned char eight_ball_index() {
  return object_balls.size();
}

/**
 * Gets the obstructing balls lying on the given line segment for a given shot. We use this function to
 * check that no balls are obstructing the path of the cue ball to the object ball and from the object
 * ball to a particular pocket. If a permanent obstruction is found, return obstructions immediately.
 * This function relies on {@code opponent_object_balls}, {@code object_balls}, {@code eight_ball}, and {@code pockets} 
 * being populated.
 *
 * @param object_ball_index the index of the object ball we are shooting at. It is not populated into the list of 
 * obstructions since it cannot obstruct itself. The eight ball is represented by an index of eight_ball_index().
 * @param consider_pockets whether or not to include the pockets on the list of possible obstructions. 
 * TODO: Make pocket handling a bit more precise.
 */
obstructions_struct get_obstructions_on_segment_for_shot(
  unsigned char object_ball_index, const Vector2d &segment_start, const Vector2d &segment_end, bool consider_pockets)
{
  obstructions_struct obstructions;
  for (unsigned char o = 0; o < opponent_object_balls.size(); ++o)
  {
    if (ball_intersects_segment(opponent_object_balls[o], segment_start, segment_end))
    {
      obstructions.has_permanent_obstruction = true;
      return obstructions;
    }
  }
  if (consider_pockets)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      if (ball_intersects_segment(pockets[p], segment_start, segment_end))
      {
        obstructions.has_permanent_obstruction = true;
        return obstructions;
      }
    }
  }
  if (object_ball_index == eight_ball_index()) {
    obstructions.has_permanent_obstruction = false;
    return obstructions;
  }
  if (ball_intersects_segment(eight_ball, segment_start, segment_end))
  {
    obstructions.has_permanent_obstruction = true;
    return obstructions;
  }
  obstructions.has_permanent_obstruction = false;
  for (unsigned char i = 0; i < object_balls.size(); ++i)
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
 * This function relies on {@code opponent_object_balls}, {@code object_balls}, {@code eight_ball}, and {@code pockets} 
 * being populated.
 */
void populate_ball_to_pocket_obstructions_table()
{
  ball_to_pocket_obstructions_table = vector<vector<obstructions_struct>>(object_balls.size() + 1, vector<obstructions_struct>(pockets.size()));
  for (unsigned char p = 0; p < pockets.size(); ++p)
  {
    Vector2d pocket = pockets[p];
    for (unsigned char o = 0; o < object_balls.size(); ++o)
    {
      ball_to_pocket_obstructions_table[o][p] = get_obstructions_on_segment_for_shot(o, object_balls[o], pocket, false);
    }
    ball_to_pocket_obstructions_table[eight_ball_index()][p] = get_obstructions_on_segment_for_shot(eight_ball_index(), eight_ball, pocket, false);
  }
}

/**
 * Calculates the ghost ball position for a given ball shot into a particular pocket.
 */
Vector2d get_ghost_ball_for_shot(const Vector2d &ball, const Vector2d &pocket)
{
  Vector2d pocket_to_ball = ball - pocket;
  return ball + pocket_to_ball.normalized() * BALL_DIAMETER;
}

/**
 * Populates the ghost ball position table. 
 * This function relies on {@code object_balls}, {@code eight_ball}, and {@code pockets} being populated.
 */
void populate_ghost_ball_position_table()
{
  ghost_ball_position_table = vector<vector<Vector2d>>(object_balls.size() + 1, vector<Vector2d>(pockets.size()));
  for (unsigned char p = 0; p < pockets.size(); ++p)
  {
    for (unsigned char o = 0; o < object_balls.size(); ++o)
    {
      ghost_ball_position_table[o][p] = get_ghost_ball_for_shot(object_balls[o], pockets[p]);
    }
    ghost_ball_position_table[eight_ball_index()][p] = get_ghost_ball_for_shot(eight_ball, pockets[p]);
  }
}

/**
 * Modifies set 1 by adding all the elements from set 2.
 */
void insert_into_set(set<unsigned char> &set1, set<unsigned char> &set2)
{
  for (unsigned char index : set2)
  {
    set1.insert(index);
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
 * Uses the method outlined here: https://www.topcoder.com/community/competitive-programming/tutorials/geometry-concepts-line-intersection-and-its-applications/
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
  segment_intersection.has_intersection = false;
  if (double_equals(det, 0))
  {
    return segment_intersection;
  }
  double x = (B2 * C1 - B1 * C2) / det;
  double y = (A1 * C2 - A2 * C1) / det;
  if (!double_equals(x, segment1_ranges.min_x) && !double_equals(x, segment1_ranges.max_x) && (x < segment1_ranges.min_x || x > segment1_ranges.max_x))
  {
    return segment_intersection;
  }
  else if (!double_equals(y, segment1_ranges.min_y) && !double_equals(y, segment1_ranges.max_y) && (y < segment1_ranges.min_y || y > segment1_ranges.max_y))
  {
    return segment_intersection;
  }
  else if (!double_equals(x, segment2_ranges.min_x) && !double_equals(x, segment2_ranges.max_x) && (x < segment2_ranges.min_x || x > segment2_ranges.max_x))
  {
    return segment_intersection;
  }
  else if (!double_equals(x, segment1_ranges.min_y) && !double_equals(x, segment1_ranges.max_y) && (y < segment2_ranges.min_y || y > segment2_ranges.max_y))
  {
    return segment_intersection;
  }
  segment_intersection.has_intersection = true;
  segment_intersection.intersection_point = Vector2d(x, y);
  return segment_intersection;
}

segment_intersection_struct get_intersection_of_line_segments(
    const Vector2d &segment1_start, const Vector2d &segment1_end,
    const vector<Vector2d> &segment2) 
{
  return get_intersection_of_line_segments(segment1_start, segment1_end, segment2[0], segment2[1]);
}

/**
 * Gets the unit tangent line vector for a given cue ball, ghost ball, and pocket and other shot angle info.
 */
shot_angle_struct get_shot_angle(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Vector2d &pocket)
{
  shot_angle_struct shot_angle;
  shot_angle.origin = ghost_ball;
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  Vector2d ghost_ball_to_pocket = pocket - ghost_ball;
  shot_angle.pocket_direction = ghost_ball_to_pocket.normalized();
  float determinant = ghost_ball_to_cue_ball.x() * ghost_ball_to_pocket.y() - ghost_ball_to_cue_ball.y() * ghost_ball_to_pocket.x();
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
  shot_angle.fractional_distance = 1 - negative_cos_theta * negative_cos_theta;
  return shot_angle;
}

/**
 * Gets the path of a ball after it bounces off table edges.
 * Depends on pocket and table edges being initialized.
 * segment_length is passed in for optimization purposes. It can also be calculated using segment_start
 * and segment_end.
 */
rail_reflection_struct reflect_ball_path_off_table_edges(
    const Vector2d &segment_start, const Vector2d &segment_end, float segment_length)
{
  rail_reflection_struct rail_reflection;
  rail_reflection.has_intersection = false;
  Vector2d direction_of_final_vector;

  if (segment_end.x() < 0)
  {
    segment_intersection_struct segment_intersection = get_intersection_of_line_segments(segment_start, segment_end, left_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(-1 * start_to_end.x(), start_to_end.y()).normalized();
      rail_reflection.has_intersection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (segment_end.x() > WIDTH)
  {
    segment_intersection_struct segment_intersection = get_intersection_of_line_segments(segment_start, segment_end, right_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(-1 * start_to_end.x(), start_to_end.y()).normalized();
      rail_reflection.has_intersection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (segment_end.y() < 0)
  {
    segment_intersection_struct segment_intersection = get_intersection_of_line_segments(segment_start, segment_end, bottom_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(start_to_end.x(), -1 * start_to_end.y()).normalized();
      rail_reflection.has_intersection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (segment_end.y() > LENGTH)
  {
    segment_intersection_struct segment_intersection = get_intersection_of_line_segments(segment_start, segment_end, top_edge);
    if (segment_intersection.has_intersection)
    {
      Vector2d start_to_end = segment_end - segment_start;
      direction_of_final_vector = Vector2d(start_to_end.x(), -1 * start_to_end.y()).normalized();
      rail_reflection.has_intersection = true;
      rail_reflection.intersection_point = segment_intersection.intersection_point;
    }
  }
  if (!rail_reflection.has_intersection)
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
 * TODO: Scale the strength to the distance non linearly.
 */
float strength_to_distance(unsigned char strength)
{
  return 1.0 * strength * MAX_DIAMONDS / NUM_STRENGTHS * UNITS_PER_DIAMOND;
}

/**
 * Gets the path given the shot angle info, a strength, and a spin.
 * Relies on pockets and table edges being populated.
 * TODO: This is just a hack right now. Does not take spin into account.
 */
vector<Vector2d> get_path(shot_angle_struct shot_angle, unsigned char strength, unsigned char spin)
{
  vector<Vector2d> ret;
  float distance_to_travel = shot_angle.fractional_distance * strength_to_distance(strength);
  Vector2d cue_ball_destination = shot_angle.tangent_line_direction * distance_to_travel + shot_angle.origin;
  Vector2d cue_ball_location = shot_angle.origin;
  ret.push_back(cue_ball_location);
  rail_reflection_struct rail_reflection = reflect_ball_path_off_table_edges(
      cue_ball_location, cue_ball_destination, distance_to_travel);
  while (rail_reflection.has_intersection)
  {
    ret.push_back(rail_reflection.intersection_point);
    cue_ball_location = rail_reflection.intersection_point;
    cue_ball_destination = rail_reflection.end_point;
    distance_to_travel = rail_reflection.distance_traveled_after_rail;
    rail_reflection = reflect_ball_path_off_table_edges(
        cue_ball_location, cue_ball_destination, distance_to_travel);
  }
  ret.push_back(cue_ball_destination);
  return ret;
}

/**
 * Populates the shot info table obstructions.
 * Relies on initialize_pockets and populate_ball_to_pocket_obstructions_table, populate_ghost_ball_table.
 */
void populate_shot_info_table_obstructions()
{
  shot_info_table = vector<vector<vector<vector<shot_info_struct>>>>(WIDTH + 1, vector<vector<vector<shot_info_struct>>>(LENGTH + 1, vector<vector<shot_info_struct>>(object_balls.size() + 1, vector<shot_info_struct>(pockets.size()))));
  for (unsigned char o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      obstructions_struct ball_to_pocket_obstructions = ball_to_pocket_obstructions_table[o][p];
      Vector2d ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned char w = 0; w <= WIDTH; ++w)
      {
        for (unsigned char l = 0; l <= LENGTH; ++l)
        {
          shot_info_struct &shot_info = shot_info_table[w][l][o][p];
          shot_info.shot_obstructions = ball_to_pocket_obstructions;
          if (ball_to_pocket_obstructions.has_permanent_obstruction)
          {
            continue;
          }
          Vector2d cue_ball = Vector2d(w, l);
          obstructions cue_to_object_obstructions = get_obstructions_on_segment_for_shot(o, cue_ball, ghost_ball_position, false);
          if (cue_to_object_obstructions.has_permanent_obstruction)
          {
            shot_info.shot_obstructions.obstructing_object_balls.clear();
            continue;
          }
          shot_info.shot_obstructions.has_permanent_obstruction = false;
          insert_into_set(shot_info.shot_obstructions.obstructing_object_balls, cue_to_object_obstructions.obstructing_object_balls);
        }
      }
    }
  }
}

/**
 * Gets the shot difficulty given a cue ball, ghost ball location, and a pocket. It does this by
 * projecting the cue ball onto the ghost ball and pocket line. It then multiplies the projected
 * distance by the distance of the ghost ball to the pocket. Represents the shot difficulty
 * stated here: http://www.sfbilliards.com/articles/1994.pdf
 * If impossible, it returns inf.
 */
float get_shot_difficulty(const Vector2d &cue_ball, const Vector2d &ghost_ball, Vector2d pocket)
{
  if (cue_ball == ghost_ball)
  {
    return 0;
  }
  Vector2d ghost_ball_to_pocket = pocket - ghost_ball;
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  float projected_distance_of_cue_ball_to_shot_line = ghost_ball_to_cue_ball.squaredNorm() * ghost_ball_to_pocket.norm() / ghost_ball_to_pocket.dot(ghost_ball_to_cue_ball);
  if (projected_distance_of_cue_ball_to_shot_line > 0)
  {
    return std::numeric_limits<float>::infinity();
  }
  return -1 * projected_distance_of_cue_ball_to_shot_line * ghost_ball_to_pocket.norm();
}

/**
 * Populates the shot info table with the difficulty of each shot. Does not take the next shot into account.
 * Simply calculates the difficulty of making the given shot.
 */
void populate_shot_info_table_difficulty()
{
  for (unsigned char o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      const Vector2d &ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned char w = 0; w <= WIDTH; ++w)
      {
        for (unsigned char l = 0; l <= LENGTH; ++l)
        {
          shot_info_struct &shot_info = shot_info_table[w][l][o][p];
          shot_info.difficulty = get_shot_difficulty(Vector2d(w, l), ghost_ball_position, pockets[p]);
          shot_info.weighted_difficulty = shot_info.difficulty * shot_info.difficulty;
        }
      }
    }
  }
}

/**
 * Populates the table of shot paths.
 * TODO: Take the spins into account when calculating the shot paths.
 * TODO: Possible optimization is to populate a map of map[object balls][pockets][angles][num strength][num spins] and use that to
 * populate the full table of map[object balls][pockets][cue ball positions][num strengths][num spins]
 */
void populate_shot_path_table()
{
  shot_path_table = vector<vector<vector<vector<vector<vector<shot_path_struct>>>>>>(WIDTH + 1, vector<vector<vector<vector<vector<shot_path_struct>>>>>(LENGTH + 1, vector<vector<vector<vector<shot_path_struct>>>>(object_balls.size() + 1, vector<vector<vector<shot_path_struct>>>(pockets.size(), vector<vector<shot_path_struct>>(NUM_STRENGTHS, vector<shot_path_struct>(NUM_SPINS))))));
  for (unsigned char o = 0; o < object_balls.size() + 1; ++o)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      const Vector2d &ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned char w = 0; w <= WIDTH; ++w)
      {
        for (unsigned char l = 0; l <= LENGTH; ++l)
        {
          shot_angle_struct shot_angle = get_shot_angle(Vector2d(w, l), ghost_ball_position, pockets[p]);
          for (unsigned char st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned char sp = 0; sp < NUM_SPINS; ++sp)
            {
              shot_path_struct &current_shot_path = shot_path_table[w][l][o][p][st][sp];
              if (shot_info_table[w][l][o][p].difficulty > MAX_DIFFICULTY_SHOT_TO_CONSIDER)
              {
                continue;
              }
              current_shot_path.within_acceptable_difficulty = true;
              vector<Vector2d> path = get_path(shot_angle, st, sp);
              current_shot_path.path_segments = path;
              current_shot_path.final_position = path[path.size() - 1];
              for (unsigned char pa = 0; pa < path.size() - 1; ++pa)
              {
                obstructions obstructions_on_segment = get_obstructions_on_segment_for_shot(o, path[pa], path[pa + 1], true);
                if (obstructions_on_segment.has_permanent_obstruction)
                {
                  break;
                }
                current_shot_path.shot_obstructions.has_permanent_obstruction = false;
                insert_into_set(current_shot_path.shot_obstructions.obstructing_object_balls, obstructions_on_segment.obstructing_object_balls);
              }
            }
          }
        }
      }
    }
  }
}

// TODO: Unit tests from here down.
void populate_eight_ball_in_selected_shot_table()
{
  for (unsigned char w = 0; w <= WIDTH; ++w)
  {
    for (unsigned char l = 0; l <= LENGTH; ++l)
    {
      selected_shot_struct &selected_shot = selected_shot_table[w][l][0];
      for (unsigned char p = 0; p < pockets.size(); ++p)
      {
        shot_info_struct &current_shot_info = shot_info_table[w][l][eight_ball_index()][p];
        if (!current_shot_info.shot_obstructions.has_permanent_obstruction && current_shot_info.difficulty <= MAX_DIFFICULTY_SHOT_TO_CONSIDER &&
            current_shot_info.weighted_difficulty < selected_shot.total_weighted_difficulty)
        {
          selected_shot.possible = true;
          selected_shot.total_weighted_difficulty = current_shot_info.weighted_difficulty;
          selected_shot.pocket = p;
          selected_shot.object_ball = eight_ball_index();
          selected_shot.strength = 0; // TODO: Handle path better
          selected_shot.spin = 0;     // TODO: Handle path better
        }
      }
    }
  }
}

// TODO: Doesn't work.
void populate_single_combination_in_selected_shot_table(int combo, set<unsigned char> &balls)
{
  for (auto ball : balls)
  {
    for (unsigned char w = 0; w <= WIDTH; ++w)
    {
      for (unsigned char l = 0; l <= LENGTH; ++l)
      {
        selected_shot_struct &selected_shot = selected_shot_table[w][l][ball];
        for (unsigned char p = 0; p < pockets.size(); ++p)
        {
          shot_info_struct &current_shot_info = shot_info_table[w][l][ball][p];
          if (!current_shot_info.shot_obstructions.has_permanent_obstruction && current_shot_info.difficulty <= MAX_DIFFICULTY_SHOT_TO_CONSIDER &&
              current_shot_info.weighted_difficulty < selected_shot.total_weighted_difficulty)
          {
            for (unsigned char st = 0; st < NUM_STRENGTHS; ++st)
            {
              for (unsigned char sp = 0; sp < NUM_SPINS; ++sp)
              {
                shot_path_struct &current_shot_path = shot_path_table[w][l][ball][p][st][sp];
                if (!current_shot_path.within_acceptable_difficulty || current_shot_path.shot_obstructions.has_permanent_obstruction)
                {
                  continue;
                }
                set<unsigned char> intersect;
                set_intersection(balls.begin(), balls.end(), current_shot_path.shot_obstructions.obstructing_object_balls.begin(), current_shot_path.shot_obstructions.obstructing_object_balls.end(),
                                 std::inserter(intersect, intersect.begin()));
                if (intersect.size() > 0)
                {
                  continue;
                }
                unsigned char rounded_x = (unsigned char)(0.5 + current_shot_path.final_position.x());
                unsigned char rounded_y = (unsigned char)(0.5 + current_shot_path.final_position.y());
                float new_weighted_difficulty = current_shot_info.weighted_difficulty + selected_shot_table[rounded_x][rounded_y][combo - (1 << ball)].total_weighted_difficulty;
                if (new_weighted_difficulty < selected_shot.total_weighted_difficulty)
                {
                  selected_shot.total_weighted_difficulty = current_shot_info.weighted_difficulty;
                  selected_shot.pocket = p;
                  selected_shot.object_ball = ball;
                  selected_shot.strength = st;
                  selected_shot.spin = sp;
                  selected_shot.path_segments = current_shot_path.path_segments;
                }
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
  set<unsigned char> balls;
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
  selected_shot_table = vector<vector<vector<selected_shot_struct>>>(WIDTH + 1, vector<vector<selected_shot_struct>>(LENGTH + 1, vector<selected_shot_struct>(object_balls.size() * object_balls.size())));
  populate_eight_ball_in_selected_shot_table();
  for (unsigned char i = 1; i <= object_balls.size(); ++i)
  {
    process_object_ball_combinations(object_balls.size(), i);
  }
  selected_shot_struct selected_shot = selected_shot_table[8][3][7];
  cout << selected_shot.object_ball << endl;
  cout << selected_shot.pocket << endl;
  cout << selected_shot.total_weighted_difficulty << endl;
}

/*
std::ostream &operator<<(std::ostream &o, const obstructions &obstructions)
{
  o << "Permanent obstruction: " << obstructions.has_permanent_obstruction << endl;
  o << "Object ball obstructions: " << endl;
  for (unsigned char index : obstructions.obstructing_object_balls)
  {
    o << (int)index << " ";
  }
  return o;
}

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
