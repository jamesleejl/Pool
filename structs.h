#ifndef STRUCTS_H
#define STRUCTS_H

#include <set>
#include "./utils.h"

/**
 * Whether or not a shot is obstructed and whether it can become unobstructed.
 */
struct BallObstructions
{
  public:
    BallObstructions() : has_permanent_obstruction_(false) {}
    void set_has_permanent_obstruction() {
      has_permanent_obstruction_ = true;
      obstructing_player_balls_.clear();
    }
    void add_obstructing_player_balls(short ball_index) {
      obstructing_player_balls_.insert(ball_index);
    }
    bool get_has_permanent_obstruction() const {
      return has_permanent_obstruction_;
    }
    const std::set<short>& get_obstructing_player_balls() const {
      return obstructing_player_balls_;
    }
    void add_obstructing_player_balls(set<short> obstructing_player_balls) {
      insert_into_set(obstructing_player_balls_, obstructing_player_balls);
    }
  private:
    /**
     * A shot is permanently obstructed if blocked by the 8 ball or opponent object balls.
     */
    bool has_permanent_obstruction_;
    /**
     * Obstructing object balls belonging to the player are listed here.
     * This is only populated if has_permanent_obstruction is false.
     */
    std::set<short> obstructing_player_balls_;
};

/**
 * Holds information about the minimum and maximum values of line segments.
 */
struct SegmentRange
{
  SegmentRange() : min_x(0), max_x(0), min_y(0), max_y(0) {}
  /**
   * The minimum x coordinate of the segments given.
   */
  double min_x;
  /**
   * The maximum x coordinate of the segments given.
   */
  double max_x;
  /**
   * The minimum y coordinate of the segments given.
   */
  double min_y;
  /**
   * The maximum y coordinate of the segments given.
   */
  double max_y;
};

/**
 * The ball position as seen by the frontend. This is the position of the ball
 * in diamonds rather than units.
 */
struct Ball
{
  public:
    Ball(double x, double y) : x_(x), y_(y) {}
    Vector2d to_vec2d() const {
      return move_ball_in_from_rails(Vector2d(from_diamonds(x_), from_diamonds(y_)));
    }
  private:
    double x_;
    double y_;
};

/**
 * Holds information about ghost balls.
 */
struct GhostBall
{
  public:
    GhostBall() : possible_(false) {}
    void set_coords(Vector2d coords) {
      coords_ = coords;
      possible_ = true;
    }
    void set_impossible() {
      coords_ = Vector2d(-1, -1);
      possible_ = false;
    }
    const Vector2d& get_coords() const {
      return coords_;
    }
    const bool get_possible() const {
      return possible_;
    }
  private:
    /**
     * Whether there is a valid ghost ball or not.
     */
    bool possible_;
    /**
     * The coordinates.
     */
    Vector2d coords_;
};

/**
 * All information related to a pool shot.
 */
struct Shot
{
  public:
    Shot() :
      possible_(false),
      ghost_ball_(nullptr),
      slow_shot_difficulty_(std::numeric_limits<double>::infinity()),
      weighted_slow_shot_difficulty_(std::numeric_limits<double>::infinity()),
      fast_shot_difficulty_(std::numeric_limits<double>::infinity()),
      weighted_fast_shot_difficulty_(std::numeric_limits<double>::infinity()) {}
    const GhostBall* get_ghost_ball() const {
      return ghost_ball_;
    }
    void set_ghost_ball(const GhostBall* ghost_ball) {
      ghost_ball_ = ghost_ball;
    }
    void set_shot_obstructions(const BallObstructions& ball_obstructions) {
      shot_obstructions_ = ball_obstructions;
    }
    const BallObstructions& get_shot_obstructions() const {
      return shot_obstructions_;
    }
    void set_possible() {
      possible_ = true;
    }
    bool get_possible() const {
      return possible_;
    }
    void set_impossible() {
      possible_ = false;
    }
    const Vector2d& get_cue_ball_to_ghost_ball() const {
      return cue_ball_to_ghost_ball_;
    }
    void set_cue_ball_to_ghost_ball(const Vector2d& cue_ball_to_ghost_ball) {
      cue_ball_to_ghost_ball_ = cue_ball_to_ghost_ball;
    }
    void set_shot_angle(double shot_angle) {
      shot_angle_ = shot_angle;
    }
    double get_shot_angle() const {
      return shot_angle_;
    }
    void set_slow_shot_difficulty(double slow_shot_difficulty) {
      slow_shot_difficulty_ = slow_shot_difficulty;
    }
    double get_slow_shot_difficulty() const {
      return slow_shot_difficulty_;
    }
    void set_weighted_slow_shot_difficulty(double weighted_slow_shot_difficulty) {
      weighted_slow_shot_difficulty_ = weighted_slow_shot_difficulty;
    }
    double get_weighted_slow_shot_difficulty() const {
      return weighted_slow_shot_difficulty_;
    }
    void set_fast_shot_difficulty(double fast_shot_difficulty) {
      fast_shot_difficulty_ = fast_shot_difficulty;
    }
    double get_fast_shot_difficulty() const {
      return fast_shot_difficulty_;
    }
    void set_weighted_fast_shot_difficulty(double weighted_fast_shot_difficulty) {
      weighted_fast_shot_difficulty_ = weighted_fast_shot_difficulty;
    }
    double get_weighted_fast_shot_difficulty() const {
      return weighted_fast_shot_difficulty_;
    }
    void add_player_balls_to_shot_obstructions(const set<short> obstructing_player_balls) {
      shot_obstructions_.add_obstructing_player_balls(obstructing_player_balls);
    }
  private:
    /**
     * Whether the shot is possible or not. All other information in this struct are only populated if this is
     * set to true.
     */
    bool possible_;
    /**
     * The vector from the cue ball to the ghost ball.
     */
    Vector2d cue_ball_to_ghost_ball_;
    /**
     * A pointer to the ghost ball for the shot.
     */
    const GhostBall* ghost_ball_;
    /**
     * All obstructions encountered during the shot no matter whether it is the cue ball to the ghost ball or
     * the object ball to the pocket. Obstructions on the cue ball path are not added here.
     */
    BallObstructions shot_obstructions_;
    /**
     * The difficulty of the shot. Does not take obstructions into account.
     */
    double slow_shot_difficulty_;
    /**
     * The weighted difficulty of the shot so that we can just add numbers together when considering runout paths.
     */
    double weighted_slow_shot_difficulty_;
    /**
     * The difficulty of the shot. Does not take obstructions into account.
     */
    double fast_shot_difficulty_;
    /**
     * The weighted difficulty of the shot so that we can just add numbers together when considering runout paths.
     */
    double weighted_fast_shot_difficulty_;
    /**
     * The angle of the shot. Represents the angle from the the initial cue ball path to the final object ball path.
     * Counterclockwise is positive.
     */
    double shot_angle_;
};

/**
 * All information related to a pocket.
 */
struct Pocket
{
  public:
    enum PocketType { CORNER, SIDE };
    Pocket(const Vector2d& position, const Vector2d& center_line, PocketType type) :
      position_(position),
      center_line_(center_line),
      type_(type) {}
    const Vector2d& get_position() const {
      return position_;
    }
    const Vector2d& get_center_line() const {
      return center_line_;
    }
    PocketType get_type() const {
      return type_;
    }
  private:
    /**
     * Where on the table the pocket is positioned.
     */
    const Vector2d position_;
    /**
     * The unit vector extending from the position of the pocket towards the table.
     * This vector divides the pocket exactly in half. All shots along this line
     * are at a zero degree angle to the pocket.
     */
    const Vector2d center_line_;
    /**
     * The type of pocket.
     */
    PocketType type_;
};

/**
 * All information related to a pool shot including after contact with the object ball.
 */
struct PostShot
{
  public:
    PostShot() :
      possible_(false),
      shot_(nullptr) {}
    void set_possible() {
      possible_ = true;
    }
    bool get_possible() {
      return possible_;
    }
    void set_impossible() {
      possible_ = false;
    }
    void set_shot(const Shot* shot) {
      shot_ = shot;
    }
    void set_shot_difficulty(double shot_difficulty) {
      shot_difficulty_ = shot_difficulty;
    }
    double get_shot_difficulty() const {
      return shot_difficulty_;
    }
    void set_weighted_shot_difficulty(double weighted_shot_difficulty) {
      weighted_shot_difficulty_ = weighted_shot_difficulty;
    }
    double get_weighted_shot_difficulty() const {
      return weighted_shot_difficulty_;
    }
    void set_cue_ball_path(const vector<Vector2d>& cue_ball_path) {
      cue_ball_path_ = cue_ball_path;
      final_position_ = cue_ball_path[cue_ball_path.size() - 1];
    }
    const vector<Vector2d>& get_cue_ball_path() const {
      return cue_ball_path_;
    }
    const Vector2d& get_final_position() {
      return final_position_;
    }
    Speed get_speed_type() const {
      return speed_type_;
    }
    void set_speed_type(Speed speed_type) {
      speed_type_ = speed_type;
    }
    const BallObstructions& get_obstructions() const {
      return obstructions_;
    }
    void add_player_balls_to_obstructions(const set<short> obstructing_player_balls) {
      obstructions_.add_obstructing_player_balls(obstructing_player_balls);
    }
  private:
    /**
     * Whether the shot is possible or not. All other information in this struct are only populated if this is
     * set to true.
     */
    bool possible_;
    /**
     * A pointer to the pre-shot information.
     */
    const Shot* shot_;
    /**
     * The line segments that make up the cue ball path after the shot is taken.
     */
    vector<Vector2d> cue_ball_path_;
    /**
     * The obstructions along the cue ball path after the shot is taken.
     */
    BallObstructions obstructions_;
    /**
     * The difficulty of the shot. Does not take obstructions into account.
     */
    double shot_difficulty_;
    /**
     * The weighted difficulty of the shot. Does not take obstructions into account.
     */
    double weighted_shot_difficulty_;
    /**
     * The final location of the cue ball along this path.
     */
    Vector2d final_position_;
    /**
     * The type of speed this is. Used to estimate effective pocket size.
     */
    Speed speed_type_;
};

/**
 * Information about reflecting a ball off a rail.
 */
struct RailIntersection
{
  public:
    RailIntersection() :
      has_intersection_(false),
      possible_(true) {}
    void set_has_intersection(bool has_intersection) {
      has_intersection_ = has_intersection;
    }
    bool has_intersection() const {
      return has_intersection_;
    }
    void set_possible(bool possible) {
      possible_ = possible;
    }
    bool get_possible() const {
      return possible_;
    }
    void set_intersection_point(const Vector2d& intersection_point) {
      intersection_point_ = intersection_point;
    }
    const Vector2d& get_intersection_point() const {
      return intersection_point_;
    }
    Edge get_intersection_edge() const {
      return intersection_edge_;
    }
    void set_intersection_edge(Edge intersection_edge) {
      intersection_edge_ = intersection_edge;
    }
    void set_scaling_factor(double scaling_factor) {
      scaling_factor_ = scaling_factor;
    }
    double get_scaling_factor() {
      return scaling_factor_;
    }
    void set_end_vector(const Vector2d& end_vector) {
      end_vector_ = end_vector;
    }
    const Vector2d& get_end_vector() const {
      return end_vector_;
    }
  private:
    /**
     * Whether or not the path intersects with the rail or not.
     */
    bool has_intersection_;
    /**
     * Whether the shot is possible or not. If it intersects a pocket, it is not.
     */
    bool possible_;
    /**
     * The intersection point with the rail.
     * Only populated if has_intersection is true.
     */
    Vector2d intersection_point_;
    /**
     * Which edge of the pool table the path intersects with.
     */
    Edge intersection_edge_;
    /**
     * Scaling factor to apply to future segments on the path.
     */
    double scaling_factor_;
    /**
     * The vector on the path from the intersection point to the new end
     * coordinate after reflection. This is relative to the intersection
     * point.
     */
    Vector2d end_vector_;
};

/**
 * Information about the shot the algorithm has chosen to shoot.
 */
struct SelectedShot
{
  public:
    SelectedShot() :
      possible_(false),
      strength_(0),
      spin_(0),
      object_ball_(0),
      pocket_(0),
      total_weighted_difficulty_(std::numeric_limits<double>::infinity()),
      current_weighted_difficulty_(std::numeric_limits<double>::infinity()),
      cue_ball_final_position_x_coordinate_(0),
      cue_ball_final_position_y_coordinate_(0),
      next_combo_(0) {}
    void set_possible(bool possible) {
      possible_ = possible;
    }
    bool get_possible() const {
      return possible_;
    }
    void set_strength(short strength) {
      strength_ = strength;
    }
    short get_strength() const {
      return strength_;
    }
    void set_spin(short spin) {
      spin_ = spin;
    }
    short get_spin() const {
      return spin_;
    }
    void set_object_ball(short object_ball) {
      object_ball_ = object_ball;
    }
    short get_object_ball() const {
      return object_ball_;
    }
    void set_pocket(short pocket) {
      pocket_ = pocket;
    }
    short get_pocket() const {
      return pocket_;
    }
    void set_total_weighted_difficulty(double total_weighted_difficulty) {
      total_weighted_difficulty_ = total_weighted_difficulty;
    }
    double get_total_weighted_difficulty() const {
      return total_weighted_difficulty_;
    }
    void set_current_weighted_difficulty(double current_weighted_difficulty) {
      current_weighted_difficulty_ = current_weighted_difficulty;
    }
    double get_current_weighted_difficulty() const {
      return current_weighted_difficulty_;
    }
    void set_cue_ball_path(const vector<Vector2d>& cue_ball_path) {
      cue_ball_path_ = cue_ball_path;
    }
    const vector<Vector2d>& get_cue_ball_path() const {
      return cue_ball_path_;
    }
    void set_expected_cue_ball_final_position(const Vector2d& expected_cue_ball_final_position) {
      expected_cue_ball_final_position_ = expected_cue_ball_final_position;
    }
    void set_cue_ball_final_position_x_coordinate(short cue_ball_final_position_x_coordinate) {
      cue_ball_final_position_x_coordinate_ = cue_ball_final_position_x_coordinate;
    }
    short get_cue_ball_final_position_x_coordinate() const {
      return cue_ball_final_position_x_coordinate_;
    }
    void set_cue_ball_final_position_y_coordinate(short cue_ball_final_position_y_coordinate) {
      cue_ball_final_position_y_coordinate_ = cue_ball_final_position_y_coordinate;
    }
    short get_cue_ball_final_position_y_coordinate() const {
      return cue_ball_final_position_y_coordinate_;
    }
    void set_next_combo(short next_combo) {
      next_combo_ = next_combo;
    }
    short get_next_combo() const {
      return next_combo_;
    }
  private:
    /**
     * Whether the shot is possible or not.
     */
    bool possible_;
    /**
     * The strength of the shot. (0..NUM_STRENGTHS-1)
     * Only populated if possible is true.
     */
    short strength_;
    /**
     * The spin of the shot. (0..NUM_SPINS-1)
     * Only populated if possible is true.
     */
    short spin_;
    /**
     * The index of the object ball to shoot. The eight ball is at 0.
     * Only populated if possible is true.
     */
    short object_ball_;
    /**
     * Which pocket index to shoot the ball into.
     * Only populated if possible is true.
     */
    short pocket_;
    /**
     * The total weighted difficulty of the runout starting from this ball.
     * Only populated if possible is true.
     */
    double total_weighted_difficulty_;
    /**
     * The weighted difficulty of the current shot from this ball.
     * Only populated if possible is true.
     */
    double current_weighted_difficulty_;
    /**
     * The points defining the line segments the cue ball travels along after contact with the object ball.
     * Only populated if possible is true.
     */
    vector<Vector2d> cue_ball_path_;
    /**
     * Expected cue ball final position
     */
    Vector2d expected_cue_ball_final_position_;
    /**
     * The next x-coordinate of the cue ball after this shot.
     * Only populated if possible is true.
     */
    short cue_ball_final_position_x_coordinate_;
    /**
     * The next y-coordinate of the cue ball after this shot.
     * Only populated if possible is true.
     */
    short cue_ball_final_position_y_coordinate_;
    /**
     * The combination of object balls for the next shot, if any.
     * Only populated if possible is true.
     */
    short next_combo_;
};

/**
 * Integer coordinates
 */
struct Coordinates
{
  public:
    Coordinates(short x, short y) :
      x_(x),
      y_(y) {}
    void set_x(short x) {
      x_ = x;
    }
    void set_y(short y) {
      y_ = y;
    }
    short get_x() const {
      return x_;
    }
    short get_y() const {
      return y_;
    }
    bool operator==(const Coordinates& coordinates) const { return x_ == coordinates.get_x() && y_ == coordinates.get_y(); }

  private:
    /**
     * The x coordinate.
     */
    short x_;
    /**
     * The y coordinate.
     */
    short y_;
};

#endif