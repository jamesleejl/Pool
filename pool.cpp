// my first program in C++
#include <iostream>
#include <vector>
#include <set>
#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

/**
 * Whether or not a shot is obstructed and whether it can become unobstructed.
 */
struct obstructions
{
  // Whether the shot is impossible by non-player balls or not.
  bool has_permanent_obstruction;
  // The set indices of of player object balls obstructing the shot.
  set<unsigned char> obstructing_object_balls;
};

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

// TODO: Unused.
struct end_location
{
  // The percentage chance of ending at this location.
  unsigned char percent_chance;
  // The coordinates representing this location.
  Vector2d coordinates;
};

// The possible information with regards to a shot
// TODO: Unused.
struct shot_info
{
  float difficulty;                            // TODO: Make this more efficient.
  set<unsigned char> obstructing_object_balls; // The object balls owned by the player that are obstructing the shot.
  obstructions obstructions;                   // THe obstructions for this shot.
};

// The length of a diamond in units.
const unsigned char DIAMOND_LENGTH = 2;
// The diameter of the balls.
const float BALL_DIAMETER = 0.18 * DIAMOND_LENGTH;
// The width of the pool table.
const unsigned char WIDTH = DIAMOND_LENGTH * 4;
// The length of the pool table.
const unsigned char LENGTH = WIDTH * 2;
// The number of pockets.
const unsigned char NUM_POCKETS = 6;
// The index number of the 8 ball.
const unsigned char EIGHT_BALL_INDEX = 7;
// The positions of the pockets.
vector<Vector2d> pockets;
// The position of the eight ball.
Vector2d eight_ball;
// The initial position of the cue ball.
Vector2d cue_ball;
// The initial position of our object balls.
vector<Vector2d> object_balls;
// The initial position of our opponent's object balls.
vector<Vector2d> opponent_object_balls;

// A table populating the difficulty of making the 8 ball shot with the cue ball at any of the
// (WIDTH, LENGTH) positions.
// TODO: Unused.
shot_info cue_ball_eight_ball_shot_info[WIDTH][LENGTH];

/**
 * A table of obstructions from each object ball index to a given pocket. Includes the 8 ball at index
 * EIGHT_BALL_INDEX.
 */
obstructions ball_to_pocket_obstructions[8][NUM_POCKETS];

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
 * Set the position of the various balls.
 * TODO: Don't hardcode this.
 */
void initialize_layout()
{
  eight_ball = Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 3);
  cue_ball = Vector2d(0.15, DIAMOND_LENGTH * 4);
  for (int i = 0; i < 7; ++i)
  {
    object_balls.push_back(Vector2d(DIAMOND_LENGTH * 1 + DIAMOND_LENGTH * 0.25 * i, DIAMOND_LENGTH * 4.15));
    opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 0.25 * i, DIAMOND_LENGTH * 3));
  }
  object_balls[3] = Vector2d(0, DIAMOND_LENGTH * 4);
}

/**
 * Returns true if the point p interescts the given segment specified by the start and end points.
 * TODO: Might never be used.
 */
bool point_intersects_segment(Vector2d p, Vector2d segment_start, Vector2d segment_end)
{
  Vector2d start_to_point = p - segment_start;
  Vector2d end_to_point = p - segment_end;
  Vector2d start_to_end = segment_end - segment_start;
  return start_to_point.dot(start_to_end) > 0 && end_to_point.dot(start_to_end) < 0;
}

/**
 * The distance from the point to the segment. Returns -1 if the perpendicular line through point
 * does not intersect with segment.
 */
double distance_from_point_to_segment(Vector2d p, Vector2d segment_start, Vector2d segment_end)
{
  Vector2d start_to_point = p - segment_start;
  Vector2d end_to_point = p - segment_end;
  Vector2d start_to_end = segment_end - segment_start;
  const float segment_length_squared = start_to_end.squaredNorm();
  const float t = start_to_point.dot(start_to_end) / segment_length_squared;
  if (t < 0 || t > 1)
  {
    return -1;
  }
  const Vector2d projection = segment_start + t * (segment_end - segment_start);
  return (p - projection).norm();
}

/**
 * Returns where a ball intersects the given line segment.
 * TODO: This calculation is slightly wrong. It ignores the case where the ball is just before or beyond
 * the line segment connecting the two points. However, this means that the ball would be either right
 * on top of the cue ball or the destination point.
 */
bool ball_intersects_segment(Vector2d ball, Vector2d segment_start, Vector2d segment_end)
{
  double distance_from_ball_to_segment = distance_from_point_to_segment(ball, segment_start, segment_end);
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
 * Gets the obstructing balls lying on the given line segment for a given shot. We use this function to
 * check that no balls are obstructing the path of the cue ball to the object ball and from the object
 * ball to a particular pocket. If a permanent obstruction is found, return obstructions immediately.
 * object_ball_index the index of the object ball we are shooting at. It is ignored in our list of obstructions.
 */
obstructions get_obstructions_on_segment_for_shot(unsigned char object_ball_index, Vector2d segment_start, Vector2d segment_end)
{
  obstructions obstructions;
  for (int i = 0; i < opponent_object_balls.size(); ++i)
  {
    if (ball_intersects_segment(opponent_object_balls[i], segment_start, segment_end))
    {
      obstructions.has_permanent_obstruction = true;
      return obstructions;
    }
  }
  if (object_ball_index != EIGHT_BALL_INDEX && ball_intersects_segment(eight_ball, segment_start, segment_end))
  {
    obstructions.has_permanent_obstruction = true;
    return obstructions;
  }
  for (int i = 0; i < object_balls.size(); ++i)
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

void populate_cue_ball_eight_ball_shot_info()
{
  for (int p = 0; p < pockets.size(); ++p)
  {
  }
  obstructions obstructing_balls = get_obstructions_on_segment_for_shot(EIGHT_BALL_INDEX, pockets[0], pockets[1]);
  for (int w = 0; w < WIDTH; ++w)
  {
    for (int l = 0; l < LENGTH; ++l)
    {
    }
  }
}

int main()
{
  initialize_pockets();
  initialize_layout();
  cout << get_obstructions_on_segment_for_shot(3, object_balls[3], pockets[3]) << endl;
}