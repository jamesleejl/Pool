#include <iostream>
#include <vector>
#include <set>
#include <chrono>
#include <ctime>

#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

// TODO: Handle bank shots.

/**
 * Whether or not a shot is obstructed and whether it can become unobstructed.
 */
struct obstructions
{
  // Whether the shot is impossible by non-player balls or not.
  bool has_permanent_obstruction = false;
  // The set indices of of player object balls obstructing the shot.
  set<unsigned char> obstructing_object_balls;
};

/**
 * Tangent line information
 */
struct tangent_line
{
  enum FollowDirection {
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

struct shot_path
{
  // The line segments that make up the shot path.
  vector<Vector2d> path_segments;
  // The obstructions along this path.
  obstructions shot_obstructions;
  // The final location of the cue ball along this path.
  Vector2d final_position;
};

// The possible information with regards to a shot
struct shot_info
{
  float difficulty = 0;      // TODO: Make this more efficient.
  obstructions shot_obstructions; // The obstructions for this shot.
};

std::ostream &operator<<(std::ostream &o, const shot_info &shot_info)
{
  o << shot_info.difficulty << endl;
  o << shot_info.shot_obstructions << endl;
  return o;
}

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
// The number of shot strengths to consider
const unsigned char NUM_STRENGTHS = 12;
// The number of shot spins to consider
const unsigned char NUM_SPINS = 5;
// The index number of the 8 ball.
const unsigned char EIGHT_BALL_INDEX = 7;
// The maximum difficulty of shot to consider shooting.
const unsigned char MAX_DIFFICULTY_SHOT_TO_CONSIDER = 200;
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
// The lines representing the edges of the table.
vector< Hyperplane<float,2> > table_edges;

/**
 * A table of obstructions from each object ball index to a given pocket. Includes the 8 ball at index
 * EIGHT_BALL_INDEX. This is only used to help populate shot_obstructions.
 */
obstructions ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX + 1][NUM_POCKETS];

/**
 * A table of ghost ball positions for a given object ball index to a pocket.
 */
Vector2d ghost_ball_position_table[EIGHT_BALL_INDEX + 1][NUM_POCKETS];

/**
 * A table of shot infos including obstruction information and difficulty.
 */
shot_info shot_info_table[WIDTH][LENGTH][EIGHT_BALL_INDEX + 1][NUM_POCKETS];

/**
 * A table of possible paths given a cue ball, object ball, pocket, strength, and spin.
 */
shot_path shot_path_table[WIDTH][LENGTH][EIGHT_BALL_INDEX + 1][NUM_POCKETS][NUM_STRENGTHS][NUM_SPINS];

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

Vector2f toVector2f(Vector2d vec) {
  return Vector2f(vec.x(), vec.y());
}

/**
 * Initialize the edges of the table.
 */
void initialize_table_edges()
{
  table_edges.push_back(Hyperplane<float,2>::Through(toVector2f(pockets[0]), toVector2f(pockets[1])));
  table_edges.push_back(Hyperplane<float,2>::Through(toVector2f(pockets[1]), toVector2f(pockets[5])));
  table_edges.push_back(Hyperplane<float,2>::Through(toVector2f(pockets[5]), toVector2f(pockets[4])));
  table_edges.push_back(Hyperplane<float,2>::Through(toVector2f(pockets[4]), toVector2f(pockets[0])));
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
  for (unsigned char i = 0; i < opponent_object_balls.size(); ++i)
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

void populate_ball_to_pocket_obstructions_table()
{
  for (unsigned char p = 0; p < pockets.size(); ++p)
  {
    Vector2d pocket = pockets[p];
    for (unsigned char o = 0; o < object_balls.size(); ++o)
    {
      ball_to_pocket_obstructions_table[o][p] = get_obstructions_on_segment_for_shot(o, object_balls[o], pocket);
    }
    ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p] = get_obstructions_on_segment_for_shot(EIGHT_BALL_INDEX, eight_ball, pocket);
  }
}

Vector2d get_ghost_ball_for_shot(Vector2d ball, Vector2d pocket)
{
  Vector2d pocket_to_ball = pocket - ball;
  return ball + pocket_to_ball.normalized() * BALL_DIAMETER;
}

void populate_ghost_ball_position_table()
{
  for (unsigned char p = 0; p < pockets.size(); ++p)
  {
    for (unsigned char o = 0; o < object_balls.size(); ++o)
    {
      ghost_ball_position_table[o][p] = get_ghost_ball_for_shot(object_balls[o], pockets[p]);
    }
    ghost_ball_position_table[EIGHT_BALL_INDEX][p] = get_ghost_ball_for_shot(eight_ball, pockets[p]);
  }
}

/**
 * Modifies set 1 by adding all the elements from set 2.
 * TODO: Untested.
 */
void insert_into_set(set<unsigned char> &set1, set<unsigned char> &set2) {
  for (unsigned char index : set2)
  {
    set1.insert(index);
  }
}

void populate_shot_info_table_obstructions()
{
  for (unsigned char o = 0; o < EIGHT_BALL_INDEX + 1; ++o)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      obstructions ball_to_pocket_obstructions = ball_to_pocket_obstructions_table[o][p];
      Vector2d ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned char w = 0; w < WIDTH; ++w)
      {
        for (unsigned char l = 0; l < LENGTH; ++l)
        {
          shot_info &shot_info = shot_info_table[w][l][o][p];
          shot_info.shot_obstructions = ball_to_pocket_obstructions;
          if (ball_to_pocket_obstructions.has_permanent_obstruction)
          {
            continue;
          }
          Vector2d cue_ball = Vector2d(w, l);
          obstructions cue_to_object_obstructions = get_obstructions_on_segment_for_shot(o, cue_ball, ghost_ball_position);
          if (cue_to_object_obstructions.has_permanent_obstruction)
          {
            shot_info.shot_obstructions.has_permanent_obstruction = true;
            shot_info.shot_obstructions.obstructing_object_balls.clear();
            continue;
          }
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
float get_shot_difficulty(Vector2d cue_ball, Vector2d ghost_ball, Vector2d pocket)
{
  Vector2d ghost_ball_to_pocket = pocket - ghost_ball;
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  float projected_distance_of_cue_ball_to_shot_line = ghost_ball_to_cue_ball.squaredNorm() * ghost_ball_to_pocket.norm() / ghost_ball_to_pocket.dot(ghost_ball_to_cue_ball);
  if (projected_distance_of_cue_ball_to_shot_line > 0)
  {
    return std::numeric_limits<float>::infinity();
  }
  return -1 * projected_distance_of_cue_ball_to_shot_line * ghost_ball_to_pocket.norm();
}

void populate_shot_info_table_difficulty()
{
  for (unsigned char o = 0; o < EIGHT_BALL_INDEX + 1; ++o)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      Vector2d ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned char w = 0; w < WIDTH; ++w)
      {
        for (unsigned char l = 0; l < LENGTH; ++l)
        {
          shot_info &shot_info = shot_info_table[w][l][o][p];
          shot_info.difficulty = get_shot_difficulty(Vector2d(w, l), ghost_ball_position, pockets[p]);
        }
      }
    }
  }
}

/**
 * Gets the unit tangent line vector for a given cue ball, ghost ball, and pocket.
 */
tangent_line get_tangent_line(Vector2d cue_ball, Vector2d ghost_ball, Vector2d pocket)
{
  tangent_line tangent;
  tangent.origin = ghost_ball;
  Vector2d ghost_ball_to_cue_ball = cue_ball - ghost_ball;
  Vector2d ghost_ball_to_pocket = pocket - ghost_ball;
  tangent.pocket_direction = ghost_ball_to_pocket.normalized();
  float determinant = ghost_ball_to_cue_ball.x() * ghost_ball_to_pocket.y() - ghost_ball_to_cue_ball.y() * ghost_ball_to_pocket.x();
  Vector2d tangent_line_direction = ghost_ball_to_pocket.unitOrthogonal();
  if (determinant < 0)
  {
    tangent.follow_direction = tangent_line::COUNTER_CLOCKWISE;
    tangent.tangent_line_direction = Vector2d(-1 * tangent_line_direction.x(), -1 * tangent_line_direction.y());
  } else {
    tangent.follow_direction = tangent_line::CLOCKWISE;
    tangent.tangent_line_direction = tangent_line_direction;
  }
  return tangent;
}

/**
 * Gets the intersection of a line segment with the edges of the table.
 */
void get_intersection_with_table_edges(Vector2d segment_start, Vector2d segment_end) {

  Hyperplane<float,2> segment = Hyperplane<float,2>::Through(toVector2f(segment_start), toVector2f(segment_end);

  for (unsigned char i = 0; i < table_edges.size(); ++i) {
    std::cout << "Intersection:\n" << segment.intersection(table_edges[0]) << '\n';
  }
}
/**
 * Gets the path given the tangent line, a strength, and a spin.
 * TODO: This is just a hack right now.
 */
vector<Vector2d> get_path(tangent_line tangent, unsigned char strength, unsigned char spin)
{
  vector<Vector2d> ret;
  Vector2d initial_end_point = (tangent.tangent_line_direction * strength * DIAMOND_LENGTH / 2) + tangent.origin;

  ret.push_back(tangent.origin);
  ret.push_back(tangent.origin);
  ret.push_back(tangent.origin);
  ret.push_back(initial_end_point);
  return ret;
}

void populate_shot_path_table()
{
  for (unsigned char o = 0; o < EIGHT_BALL_INDEX + 1; ++o)
  {
    for (unsigned char p = 0; p < pockets.size(); ++p)
    {
      Vector2d ghost_ball_position = ghost_ball_position_table[o][p];
      for (unsigned char w = 0; w < WIDTH; ++w)
      {
        for (unsigned char l = 0; l < LENGTH; ++l)
        {
          tangent_line tangent = get_tangent_line(Vector2d(w, l), ghost_ball_position, pockets[p]);
          for (unsigned char st = 0; st < NUM_STRENGTHS; ++st)
          {
            for (unsigned char sp = 0; sp < NUM_SPINS; ++sp)
            {
              shot_path& current_shot_path = shot_path_table[w][l][o][p][st][sp];
              if (shot_info_table[w][l][o][p].difficulty > MAX_DIFFICULTY_SHOT_TO_CONSIDER)
              {
                continue;
              }
              vector<Vector2d> path = get_path(tangent, st, sp);
              current_shot_path.path_segments = path;
              current_shot_path.final_position = path[path.size() - 1];
              for (unsigned char pa = 0; pa < path.size() - 1; ++pa) {
                obstructions obstructions_on_segment = get_obstructions_on_segment_for_shot(o, path[pa], path[pa + 1]);
                if (obstructions_on_segment.has_permanent_obstruction) {
                  current_shot_path.shot_obstructions.has_permanent_obstruction = true;
                  break;
                }
                insert_into_set(current_shot_path.shot_obstructions.obstructing_object_balls, obstructions_on_segment.obstructing_object_balls);
              }
            }
          }
        }
      }
    }
  }
}

void populate_shot_info_table()
{
  populate_shot_info_table_obstructions();
  populate_shot_info_table_difficulty();
  populate_shot_path_table();
}

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
  populate_shot_info_table();

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