#ifndef POOL_H
#define POOL_H

#include <vector>
#include <set>
#include "Eigen/Dense"
#include "./constants.h"
#include "./precomputed.h"
using namespace Eigen;
using namespace std;

struct SegmentRange;
struct BallObstructions;
struct Ball;
struct GhostBall;
struct Shot;
struct Pocket;
struct PostShot;
struct RailIntersection;
struct SelectedShot;
struct Coordinates;
struct Solution;

/**
 * The positions of the pockets.
 */
extern vector<Pocket> pockets;
/**
 * The position of the cue ball.
 */
extern Vector2d cue_ball;
/**
 * The position of the eight ball.
 */
extern Vector2d eight_ball;
/**
 * The position of the player's object balls. The eight ball is at position
 * {@code eight_ball_index}. All others are the player's balls.
 */
extern vector<Vector2d> player_balls;
/**
 * The position of the player's object balls without the 8 ball.
 */
extern vector<Vector2d> player_balls_without_eight_ball;
/**
 * The position of the opponent's object balls.
 */
extern vector<Vector2d> opponent_balls;
/**
 * The bottom edge of the pool table.
 */
extern vector<Vector2d> bottom_edge;
/**
 * The right edge of the pool table.
 */
extern vector<Vector2d> right_edge;
/**
 * The top edge of the pool table.
 */
extern vector<Vector2d> top_edge;
/**
 * The left edge of the pool table.
 */
extern vector<Vector2d> left_edge;
/**
 * A table of obstructions from each player's balls to a given pocket. Includes the 8 ball at index
 * 0 and all other of the player's balls starting at position 1.
 * Dimensions are [Num player balls + 1][Num pockets]
 */
extern vector<vector<BallObstructions>> player_ball_to_pocket_obstructions_table;
/**
 * A table of ghost ball positions for a given object ball index to a pocket. Includes the 8 ball at index
 * 0 and all other of the player's balls starting at position 1.
 * Dimensions are [Num player balls + 1][Num pockets]
 */
extern vector<vector<GhostBall>> ghost_ball_position_table;
/**
 * A table of shot information including obstruction information and difficulty. Includes the 8 ball at index
 * 0 and all other of the player's balls starting at position 1.
 * Dimensions are [Length + 1][Width + 1][Num player balls + 1][Num pockets]
 */
extern vector<vector<vector<vector<Shot>>>> shot_table;
/**
 * A table of possible paths given a cue ball, object ball, pocket, strength, and spin. Includes the 8 ball at index
 * 0 and all other of the player's balls starting at position 1.
 * Dimensions are [Length + 1][Width + 1][Num player balls + 1][Num pockets][Num strengths][Num spins].
 */
extern vector<vector<vector<vector<vector<vector<PostShot>>>>>> post_shot_table;
/**
 * Gets the best shot path to for each given cue ball position for the given layout.
 * Dimensions are [Width][Length][Num table layouts]. The table layout index is generated
 * by summing the (2^object ball indices) of the remaining balls. That is, if there are
 * 3 object balls, their indices will be 1,2,4. A table which consists of balls 1 and
 * 3 will be in position 5. The table with only the 8 ball remaining is at position 0.
 * Dimensions are [Length + 1][Width + 1][Num table layouts].
 */
extern vector<vector<vector<SelectedShot>>> selected_shot_table;
/**
 * Initialize the pool table with balls and table info.
 */
void initialize(
  const Ball& cue_ball_in_diamonds,
  const Ball& eight_ball_in_diamonds,
  const vector<Ball>& player_balls_in_diamonds,
  const vector<Ball>& opponent_balls_in_diamonds);
/**
 * Populates all the tables with the information from the algorithm. The bulk of
 * the program.
 */
void populate_tables();

// ---------------------------------------------- Util functions ------------------------------------------------

/**
 * The distance from the point to the segment. Returns infinity if the perpendicular distance from the point to
 * the line defined by the segment lies off the segment. (To either side of it.)
 */
double distance_from_point_to_segment(const Vector2d &p, const Vector2d &segment_start, const Vector2d &segment_end);
/**
 * Gets the bounding box of a given line segment.
 */
SegmentRange get_segment_range(const Vector2d &segment_start, const Vector2d &segment_end);
/**
 * Returns whether a ball traveling along the path intersects the given ball.
 * Note that this function is also used with pockets passed into the {@code ball} variable. That is,
 * the pocket is modeled as a ball.
 */
bool get_ball_intersects_ball_path(const Vector2d &ball, const Vector2d &path_start, const Vector2d &path_end);
/**
 * Calculates the ghost ball position for a given ball shot into a particular pocket.
 */
GhostBall get_ghost_ball_for_shot(const Vector2d &ball, const Pocket &pocket);
/**
 * Balls cannot be within BALL_RADIUS distance of the rail. Because the ball is not a point,
 * we must push them away from the rail by the ball's radius. This returns the ball's position after
 * this operation.
 */
Vector2d move_ball_in_from_rails(const Vector2d& position);
/**
 * Gets the margin of error for a given shot given a cue ball, ghost ball location, and a pocket.
 * Utilizes https://billiards.colostate.edu/technical_proofs/TP_3-4.pdf.
 * Does not take obstructions into account.
 * Returns inf for impossible shots.
 * For shots that are closer than 1/4 diamond length away from the ghost ball, the shot
 * is said to be impossible. This could be tweaked.
 */
double get_margin_of_error_for_shot(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Pocket& pocket, Speed speed);
/**
 * Gets the difficulty of a shot.
 */
double get_shot_difficulty(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Pocket& pocket, Speed speed);
/**
 * Gets the weighted difficulty of a shot given a difficulty. Weighted difficulties are summed to
 * get runout difficulty.
 */
double get_weighted_shot_difficulty(double shot_difficulty);

/**
 * Converts from radians to 'decidegrees' which are tenths of degrees. This is for
 * lookup in the precomputed tables.
 */
short radians_to_decidegrees(double radians);
/**
 * Converts degrees to radians.
 */
double degrees_to_radians(double degrees);
/**
 * Converts radians to degrees.
 */
double radians_to_degrees(double radians);
/**
 * Gives the angle between the two vectors. The order of the vectors matters. The angle returned is
 * the direction of travel to get from vector 1 to vector 2. Positive represents counter clockwise.
 * Answer is in radians.
 */
double angle_between_vectors(const Vector2d& vec1, const Vector2d& vec2);
/**
 * Solves the quadratic equation with the given coefficients.
 */
double solve_quadratic_equation(double a, double b, double c);
/**
 * Squares the given number.
 */
double square(double num);
/**
 * Converts the strength given to the initial speed of the cue ball if it does not
 * contact an object ball. (In meters per second). Strength is a number from 0 to
 * {@code NUM_STRENGTHS} exclusive.
 * TODO: Figure out a good way to scale the strengths.
 * For reference, from Dr Dave:
 * Cue ball speeds
 * soft touch: <1 mph = <1.5 fps = < 18 ips = < 45 cm/s
 * slow: 1-2 mph = 1.5-2.9 fps = 18-35 ips = 45-89 cm/s
 * medium: 2-4 mph = 2.9-5.9 fps = 0.89-1.9 m/s
 * fast: 4-7 mph = 5.9-10.3 fps = 1.9-3.2 m/s
 * power shot: 7-10 mph = 10.3-14.7 fps = 3.2-4.5 m/s
 * powerful break: 25-30 mph = 37-44 fps = 11-13 m/s
 */
double strength_to_speed(short strength);
/**
 * Gets the speed type for a given speed. (in m/s)
 */
Speed speed_to_speed_type(double speed);
/**
 * Given a cue ball speed, get the angular speed based on the type of spin applied.
 * Cue ball speed in meters per second and the angular speed also.
 */
double get_angular_speed(double cue_ball_speed, Spin spin);
/**
 * Gets whether the segment defined by start and end points intersects any rail.
 */
RailIntersection get_rail_intersection(Vector2d start, Vector2d end);
/**
 * Apply a reflection of the point across the edge.
 */
Vector2d apply_reflection(Vector2d point, Edge edge);
/**
 * This function reflects the given path across the table rails and returns the
 * new path. If the path is empty, then it is impossible.
 *
 * path is a sequence of points defining a path which starts at the first
 * Vector2d, travels in a straight line to the next, etc.
 */
vector<Vector2d> get_path_with_reflections(vector<Vector2d> path);
/**
 * Generates a vector of numbers. Each number represents a combinations of k elements
 * from the set of n elements. These are in binary notation. That is 6 = 110 which means
 * the second and third elements are chosen. 1 = 1 which means the first ball is chosen.
 * 0 is only utilized if k = 0.
 */
vector<int> generate_all_combinations(short n, short k);
/**
 * Gets all possible combinations of k elements from the range (1..c) inclusive.
 */
void process_object_ball_combinations(int num_object_balls, int num_elements);
/**
 * Given a number representing a combination from {@code generate_all_combinations},
 * generate the set of numbers representing that combination.
 * The set of numbers returned corresponds to a 1 based indexing of balls so if the
 * combo is '1', then the number returned will be '1'. If the combo is 6, then the
 * numbers returned are '2' and '3'.
 */
set<short> get_set_from_combination(int n, int combo);
/**
 * Gets all the integral coordinates of every integral x value along the line
 * segment running from start to end. The y coordinates are rounded.
 */
vector<Coordinates> get_coordinates_between_points(Vector2d start, Vector2d end);
/**
 * Converts the speed (in m/s) to distance traveled in units.
 */
double speed_to_distance(double speed);
/**
 * Converts the speed (in m/s) of the cue ball to the speed of the object ball (in m/s). shot_angle is in radians.
 * The angle of the shot. Represents the angle from the the initial cue ball path to the final object ball path.
 * Counterclockwise is positive.
 */
double cue_ball_speed_to_object_ball_speed(double speed, double shot_angle);
/**
 * The angle between the x-axis and the line given the slope. The angle is positive when the slope is positive.
 */
double slope_to_angle(double slope);
/**
 * Gets the angle between a given angle and the edge specified. This is always positive. The angle given is
 * computed from slope_to_angle and represents the angle of a line to the x-axis.
 */
double absolute_angle_to_edge(double angle, Edge edge);
/**
 * The scaling factor to apply to ball paths after reflection from a rail given an angle to the given rail edge.
 */
double rail_path_scaling_factor_given_absolute_angle_to_edge(double absolute_angle_to_edge);
/**
 * Given a point, reflect it in the x-axis.
 */
Vector2d reflect_point_in_x_axis(const Vector2d& point);
/**
 * Given a point, reflect it in the y-axis.
 */
Vector2d reflect_point_in_y_axis(const Vector2d& point);
/**
 * Generates random ball coordinates
 * TODO: Unit test.
 */
Ball generate_random_ball();
/**
 * Returns whether the balls are far enough apart. That is, they must be at least BALL_DIAMETER apart.
 * TODO: Unit test.
 */
bool balls_far_enough_apart(const Ball& ball1, const Ball& ball2);
// -------------------------- Relies on player_balls population --------------------------------------------
// Player balls is populated with the eight ball at position 0 and all other balls from position 1 onwards.
/**
 * Gets the index of the eight ball within the {@code player_balls} vector.
 */
short eight_ball_index();
/**
 * Gets the obstructing balls in the path of a ball rolling from ball_path_start to ball_path_end.
 * This can be used to get obstructions for 3 possible cases:
 * 1. Cue ball to specified object ball.
 * 2. Specified object ball to pocket
 * 3. Cue ball path after contact with object ball. This last case is represented by
 *     {@code is_cue_ball_after_contact being true}. This path is different than the other two
 *     because we must check to see if
 *
 * @param object_ball_index the index of the object ball we are getting obstructions for. It is
 * ignored in the calculations. Note that this could mean the cue ball rolling to the given
 * object ball or the given object ball rolling to the pocket.
 * @param is_cue_ball_after_contact whether this is case 3 or not.
 * Relies on: {@code pockets}, {@code eight_ball}, {@code player_balls}, {@opponent_balls}
 */
BallObstructions get_obstructions_on_ball_path_for_ball_index(
    short object_ball_index,
    const Vector2d &ball_path_start,
    const Vector2d &ball_path_end,
    bool is_cue_ball_after_contact);
/**
 * Populates the ball_to_pocket_obstructions_table.
 * Relies on: {@code pockets}, {@code eight_ball}, {@code player_balls}, {@opponent_balls}.
 */
void populate_player_ball_to_pocket_obstructions_table();
/**
 * Populates the ghost ball position table.
 * Relies on: {@code pockets}, {@code player_balls}, {@code bottom_edge}, {@code left_edge},
 * {@code right_edge}, {@code top_edge}.
 */
void populate_ghost_ball_position_table();
/**
 * Populates the table of shots.
 * Relies on: {@code pockets}, {@code player_balls}, {@code bottom_edge}, {@code left_edge},
 * {@code right_edge}, {@code top_edge}, {@code eight_ball}, {@code opponent_balls},
 * {@code player_ball_to_pocket_obstructions_table}, {@code ghost_ball_position_table}.
 */
void populate_shot_table_obstructions();
/**
 * Populates the shot info table with the difficulty of each shot. Does not take the next shot into account.
 * Simply calculates the difficulty of making the given shot.
 * Relies on: {@code pockets}, {@code player_balls}, {@code bottom_edge}, {@code left_edge},
 * {@code right_edge}, {@code top_edge}, {@code eight_ball}, {@code opponent_balls},
 * {@code player_ball_to_pocket_obstructions_table}, {@code ghost_ball_position_table}, {@code shot_table}.
 * Note that shot table only needs to be populated with {@code ghost_ball_} and {@code shot_obstructions_}
 * thus far.
 */
void populate_shot_table_difficulty();
/**
 * Gets the effective pocket size of a pocket given a ghost ball position.
 * Relies on: {@code pockets}.
 */
double get_effective_pocket_size(const Vector2d &ghost_ball, const Pocket& pocket, Speed speed);
/**
 * Gets the angle of the shot. This is a directed angle from initial cue ball shot path to the
 * object ball travel path.
 * Relies on: {@code pockets}.
 */
double get_shot_angle(const Vector2d &cue_ball, const Vector2d &ghost_ball, const Pocket& pocket);
/**
 * Gets the cue ball path for the given shot, strength, and spin. The first element is the initial
 * cue ball position. Each subsequent vector is a directional vector to apply as a transform.
 * Relies on: {@code pockets}, {@code player_balls}, {@code bottom_edge}, {@code left_edge},
 * {@code right_edge}, {@code top_edge}, {@code eight_ball}, {@code opponent_balls},
 * {@code player_ball_to_pocket_obstructions_table}, {@code ghost_ball_position_table}, {@code shot_table}.
 * Implements https://billiards.colostate.edu/technical_proofs/new/TP_A-4.pdf and
 * https://billiards.colostate.edu/technical_proofs/new/TP_B-5.pdf.
 */
vector<Vector2d> get_cue_ball_path(const Shot& shot, short strength, Spin spin);
/**
 * Populates the post shot info table. This includes the cue ball path after taking a shot.
 * Relies on: {@code pockets}, {@code player_balls}, {@code bottom_edge}, {@code left_edge},
 * {@code right_edge}, {@code top_edge}, {@code eight_ball}, {@code opponent_balls},
 * {@code player_ball_to_pocket_obstructions_table}, {@code ghost_ball_position_table}, {@code shot_table}.
 */
void populate_post_shot_table();
/**
 * Populates the selected shot table.
 * Relies on everything.
 */
void populate_selected_shot_table();
/**
 * Returns a solution if one exists.
 * Relies on everything.
 */
Solution get_solution();
/**
 * Writes the string to the default filename defined in constants.
 */
void write_to_file(string json);
/**
 * Given a object ball speed (in m/s), calculate whether the ball can be shot into the given pocket.
 */
bool get_object_ball_shot_possible_from_object_ball_speed(double object_ball_speed, const Vector2d& ball, const Pocket& pocket);
/**
 * Given a cue ball speed (in m/s), and a shot angle, calculate whether the ball can be shot into the given pocket. Shot angle in radians.
 * The angle of the shot. Represents the angle from the the initial cue ball path to the final object ball path.
 * Counterclockwise is positive.
 */
bool get_object_ball_shot_possible_from_cue_ball_speed(double cue_ball_speed, double shot_angle, const Vector2d& ball, const Pocket& pocket);

#endif