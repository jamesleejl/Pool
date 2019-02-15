#include "./pool.h"
#include "./structs.h"
#include "./constants.h"
#include "./utils.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <iterator>
#include <algorithm>

using ::testing::ElementsAre;

namespace {

class PoolTest : public ::testing::Test
{
protected:
  PoolTest() {}

  ~PoolTest() override {}

  void SetUp() override
  {
    initialize(
      Ball(4, 2), /* Cue ball */
      Ball(2, 1), /* Eight ball */
      {}, /* Player balls */
      {} /* Opponent balls */
    );
  }

  void TearDown() override {}
};

#define EXPECT_VEC2D_EQ(vec1, vec2) {\
  EXPECT_NEAR(vec1.x(), vec2.x(), EPSILON);\
  EXPECT_NEAR(vec1.y(), vec2.y(), EPSILON);\
}
TEST_F(PoolTest, GetCoordinatesBetweenPoints) {
  vector<Coordinates> coordinates =
    get_coordinates_between_points(Vector2d(-0.1, 0.1), Vector2d(9.9, 4.1));
  EXPECT_THAT(coordinates, ElementsAre(
    Coordinates(0, 0),
    Coordinates(1, 1),
    Coordinates(2, 1),
    Coordinates(3, 1),
    Coordinates(4, 2),
    Coordinates(5, 2),
    Coordinates(6, 3),
    Coordinates(7, 3),
    Coordinates(8, 3),
    Coordinates(9, 4)));
}

TEST_F(PoolTest, GetSetFromCombination5) {
  EXPECT_THAT(get_set_from_combination(3, 5), ElementsAre(0, 2));
}

TEST_F(PoolTest, GetSetFromCombination) {
  EXPECT_THAT(get_set_from_combination(3, 6), ElementsAre(1, 2));
}

TEST_F(PoolTest, GenerateAllCombinations) {
  vector<int> combinations = generate_all_combinations(4, 1);
  EXPECT_THAT(combinations, ElementsAre(1, 2, 4, 8));
  combinations = generate_all_combinations(4, 2);
  EXPECT_THAT(combinations, ElementsAre(3, 5, 6, 9, 10, 12));
}

TEST_F(PoolTest, GetPathWithNoIntersections) {
  vector<Vector2d> path;
  path.push_back(Vector2d(2, 12));
  path.push_back(Vector2d(3, 15));
  vector<Vector2d> reflected_path = get_path_with_reflections(path);
  EXPECT_EQ(2, reflected_path.size());
  EXPECT_VEC2D_EQ(Vector2d(2, 12), reflected_path[0]);
  EXPECT_VEC2D_EQ(Vector2d(3, 15), reflected_path[1]);
}

TEST_F(PoolTest, GetPathWithReflections) {
  vector<Vector2d> path;
  path.push_back(Vector2d(2, 12));
  path.push_back(Vector2d(0, 20));
  path.push_back(Vector2d(36, 4));
  vector<Vector2d> reflected_path = get_path_with_reflections(path);
  EXPECT_EQ(8, reflected_path.size());
  EXPECT_VEC2D_EQ(Vector2d(2, 12), reflected_path[0]);
  EXPECT_VEC2D_EQ(Vector2d(1.09, 15.64), reflected_path[1]);
  EXPECT_VEC2D_EQ(Vector2d(0.36, 12.72), reflected_path[2]);
  EXPECT_VEC2D_EQ(Vector2d(0.72, 11.28), reflected_path[3]);
  EXPECT_VEC2D_EQ(Vector2d(0.36, 12.72), reflected_path[4]);
  EXPECT_VEC2D_EQ(Vector2d(7.5075824175824168, 15.64), reflected_path[5]);
  EXPECT_VEC2D_EQ(Vector2d(31.64, 5.7811896745230076), reflected_path[6]);
  EXPECT_VEC2D_EQ(Vector2d(27.28, 4), reflected_path[7]);
}

TEST_F(PoolTest, ApplyReflectionLeft) {
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(BALL_RADIUS_IN_DIAMONDS - 1), diamonds(1)),
    apply_reflection(
      Vector2d(diamonds(1 + BALL_RADIUS_IN_DIAMONDS), diamonds(1)),
      Edge::LEFT));
}

TEST_F(PoolTest, ApplyReflectionRight) {
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(3), diamonds(1)),
    apply_reflection(
      Vector2d(diamonds(13 - 2 * BALL_RADIUS_IN_DIAMONDS), diamonds(1)),
      Edge::RIGHT));
}

TEST_F(PoolTest, ApplyReflectionBottom) {
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(1), diamonds(BALL_RADIUS_IN_DIAMONDS - 1)),
    apply_reflection(
      Vector2d(diamonds(1), diamonds(1 + BALL_RADIUS_IN_DIAMONDS)),
      Edge::BOTTOM));
}

TEST_F(PoolTest, ApplyReflectionTOP) {
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(1), diamonds(7 - 2 * BALL_RADIUS_IN_DIAMONDS)),
    apply_reflection(
      Vector2d(diamonds(1), diamonds(1)),
      Edge::TOP));
}

TEST_F(PoolTest, GetRailIntersectionNoIntersection)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(1), diamonds(1)),
      Vector2d(diamonds(2), diamonds(2)));
  EXPECT_FALSE(rail_intersection.has_intersection());
}

TEST_F(PoolTest, GetRailIntersectionRightEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(7 - BALL_RADIUS_IN_DIAMONDS), diamonds(2)),
      Vector2d(diamonds(9 - BALL_RADIUS_IN_DIAMONDS), diamonds(3)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::RIGHT, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(8 - BALL_RADIUS_IN_DIAMONDS), diamonds(2.5)),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionLeftEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(1 + BALL_RADIUS_IN_DIAMONDS), diamonds(2)),
      Vector2d(diamonds(-1 + BALL_RADIUS_IN_DIAMONDS), diamonds(3)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::LEFT, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(BALL_RADIUS_IN_DIAMONDS), diamonds(2.5)),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionTopEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(2), diamonds(3 - BALL_RADIUS_IN_DIAMONDS)),
      Vector2d(diamonds(3), diamonds(5 - BALL_RADIUS_IN_DIAMONDS)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::TOP, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(2.5), WIDTH - diamonds(BALL_RADIUS_IN_DIAMONDS)),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionBottomEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(2), diamonds(1 + BALL_RADIUS_IN_DIAMONDS)),
      Vector2d(diamonds(3), diamonds(-1 + BALL_RADIUS_IN_DIAMONDS)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::BOTTOM, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(2.5), diamonds(BALL_RADIUS_IN_DIAMONDS)),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionBothBottomAndLeftEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(1 + BALL_RADIUS_IN_DIAMONDS), diamonds(1 + BALL_RADIUS_IN_DIAMONDS)),
      Vector2d(diamonds(0), diamonds(0)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::LEFT, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(diamonds(BALL_RADIUS_IN_DIAMONDS), diamonds(BALL_RADIUS_IN_DIAMONDS)),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionBottomBeforeLeftEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(1 + 2 * BALL_RADIUS_IN_DIAMONDS), diamonds(1 + BALL_RADIUS_IN_DIAMONDS)),
      Vector2d(diamonds(0), diamonds(0)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::BOTTOM, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(0.38972477064220179, BALL_RADIUS),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionLeftBeforeBottomEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(1 + BALL_RADIUS_IN_DIAMONDS), diamonds(1 + 2 * BALL_RADIUS_IN_DIAMONDS)),
      Vector2d(diamonds(0), diamonds(0)));
  EXPECT_TRUE(rail_intersection.has_intersection());
  EXPECT_EQ(Edge::LEFT, rail_intersection.get_intersection_edge());
  EXPECT_VEC2D_EQ(
    Vector2d(BALL_RADIUS, 0.38972477064220179),
    rail_intersection.get_intersection_point());
}

TEST_F(PoolTest, GetRailIntersectionOnEdge)
{
  RailIntersection rail_intersection =
    get_rail_intersection(
      Vector2d(diamonds(1), diamonds(2)),
      Vector2d(BALL_RADIUS, diamonds(2)));
  EXPECT_FALSE(rail_intersection.has_intersection());
}

TEST_F(PoolTest, GetCueBallPath)
{
  initialize(
    Ball(0, 4),
    Ball(0, 4),
    {Ball(2, 2), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4)},
    {Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4)}
  );
  populate_player_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_table_obstructions();
  populate_shot_table_difficulty();
  vector<Vector2d> cue_ball_path = get_cue_ball_path(shot_table[9][12][1][0], 9, Spin::HEAVY_FOLLOW);
  EXPECT_EQ(3, cue_ball_path.size());
  EXPECT_VEC2D_EQ(Vector2d(8.5091168824543146, 8.5091168824543146), cue_ball_path[0]);
  EXPECT_VEC2D_EQ(Vector2d(27.243889307579117, -21.344478715891714), cue_ball_path[1]);
  EXPECT_VEC2D_EQ(Vector2d(44.368176295062106, -65.643628781127632), cue_ball_path[2]);
}

TEST_F(PoolTest, Square) {
  EXPECT_DOUBLE_EQ(25, square(5));
  EXPECT_DOUBLE_EQ(1.44, square(1.2));
}

TEST_F(PoolTest, StrengthToSpeed) {
  EXPECT_DOUBLE_EQ(MIN_CUE_BALL_SPEED, strength_to_speed(0));
  EXPECT_DOUBLE_EQ(MAX_CUE_BALL_SPEED, strength_to_speed(NUM_STRENGTHS - 1));
}

TEST_F(PoolTest, SpeedToSpeedType) {
  EXPECT_EQ(Speed::SLOW, speed_to_speed_type(1.4));
  EXPECT_EQ(Speed::FAST, speed_to_speed_type(1.6));
}

TEST_F(PoolTest, GetAngularSpeed) {
  EXPECT_DOUBLE_EQ(5 * 1.25/BALL_RADIUS_IN_METERS, get_angular_speed(5, Spin::HEAVY_DRAW));
  EXPECT_DOUBLE_EQ(6 * 0.7/BALL_RADIUS_IN_METERS, get_angular_speed(6, Spin::LIGHT_DRAW));
  EXPECT_DOUBLE_EQ(-5 * 1.25/BALL_RADIUS_IN_METERS, get_angular_speed(5, Spin::HEAVY_FOLLOW));
  EXPECT_DOUBLE_EQ(-6 * 0.7/BALL_RADIUS_IN_METERS, get_angular_speed(6, Spin::LIGHT_FOLLOW));
  EXPECT_DOUBLE_EQ(0, get_angular_speed(-6, Spin::STUN));
}

TEST_F(PoolTest, WeightedShotDifficulty)
{
  EXPECT_DOUBLE_EQ(25, get_weighted_shot_difficulty(5));
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), get_weighted_shot_difficulty(std::numeric_limits<double>::infinity()));
}

TEST_F(PoolTest, RadiansToDecidegrees)
{
  EXPECT_EQ(1800, radians_to_decidegrees(M_PI));
  EXPECT_EQ(-824, radians_to_decidegrees(-1.4378));
}

TEST_F(PoolTest, DegreesToRadians)
{
  EXPECT_EQ(M_PI / 4, degrees_to_radians(45));
  EXPECT_EQ(M_PI / 6, degrees_to_radians(30));
}

TEST_F(PoolTest, RadiansToDegrees)
{
  EXPECT_DOUBLE_EQ(45, radians_to_degrees(M_PI / 4));
  EXPECT_DOUBLE_EQ(30, radians_to_degrees(M_PI / 6));
}

TEST_F(PoolTest, AngleBetweenVectors) {
  EXPECT_DOUBLE_EQ(0.78539816339744839, angle_between_vectors(Vector2d(1, 1), Vector2d(0, 3)));
  EXPECT_DOUBLE_EQ(-0.78539816339744839, angle_between_vectors(Vector2d(0, 3), Vector2d(1, 1)));
  EXPECT_DOUBLE_EQ(1.5707963267948966, angle_between_vectors(Vector2d(1, 2), Vector2d(-2, 1)));
}

TEST_F(PoolTest, SolveQuadraticEquation) {
  EXPECT_DOUBLE_EQ(0.5, solve_quadratic_equation(2, 9, -5));
}

TEST_F(PoolTest, GetEffectivePocketSize) {
  EXPECT_DOUBLE_EQ(from_diamonds(0.187753883472501), get_effective_pocket_size(Vector2d(diamonds(3), diamonds(3)), pockets[0], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(diamonds(2), 0), pockets[0], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(0, diamonds(2)), pockets[0], Speed::FAST));

  EXPECT_DOUBLE_EQ(from_diamonds(0.229879705366036), get_effective_pocket_size(Vector2d(diamonds(4), diamonds(2)), pockets[1], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.0724528999488282), get_effective_pocket_size(Vector2d(diamonds(2), diamonds(2)), pockets[1], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.0724528999488282), get_effective_pocket_size(Vector2d(diamonds(6), diamonds(2)), pockets[1], Speed::FAST));

  EXPECT_DOUBLE_EQ(from_diamonds(0.187753883472501), get_effective_pocket_size(Vector2d(diamonds(7), diamonds(1)), pockets[2], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(diamonds(7), 0), pockets[2], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(diamonds(8), diamonds(3)), pockets[2], Speed::FAST));

  EXPECT_DOUBLE_EQ(from_diamonds(0.187753883472501), get_effective_pocket_size(Vector2d(diamonds(1), diamonds(3)), pockets[3], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(diamonds(2), diamonds(4)), pockets[3], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(0, diamonds(1)), pockets[3], Speed::FAST));

  EXPECT_DOUBLE_EQ(from_diamonds(0.229879705366036), get_effective_pocket_size(Vector2d(diamonds(4), diamonds(1)), pockets[4], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.0724528999488282), get_effective_pocket_size(Vector2d(diamonds(1), diamonds(1)), pockets[4], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.0724528999488282), get_effective_pocket_size(Vector2d(diamonds(5), diamonds(3)), pockets[4], Speed::FAST));

  EXPECT_DOUBLE_EQ(from_diamonds(0.187753883472501), get_effective_pocket_size(Vector2d(diamonds(7), diamonds(3)), pockets[5], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(diamonds(8), diamonds(1)), pockets[5], Speed::FAST));
  EXPECT_DOUBLE_EQ(from_diamonds(0.113466436534962), get_effective_pocket_size(Vector2d(diamonds(6), diamonds(4)), pockets[5], Speed::FAST));

}

TEST_F(PoolTest, GetShotAngle) {
  EXPECT_DOUBLE_EQ(0.78539816339744839, get_shot_angle(Vector2d(diamonds(3), diamonds(1)), Vector2d(diamonds(1), diamonds(1)), pockets[0]));
  EXPECT_DOUBLE_EQ(-0.78539816339744839, get_shot_angle(Vector2d(diamonds(1), diamonds(3)), Vector2d(diamonds(1), diamonds(1)), pockets[0]));
  EXPECT_DOUBLE_EQ(0, get_shot_angle(Vector2d(diamonds(4), diamonds(3)), Vector2d(diamonds(4), diamonds(2)), pockets[1]));
  EXPECT_DOUBLE_EQ(-3.1415926535897931, get_shot_angle(Vector2d(diamonds(4), diamonds(1)), Vector2d(diamonds(4), diamonds(3)), pockets[1]));
}

TEST_F(PoolTest, GetShotDifficulty) {
  Vector2d ghost_ball = Vector2d(diamonds(4), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(0));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(473.71831834915463, get_shot_difficulty(cue_ball, ghost_ball, pockets[0], Speed::FAST));
}

TEST_F(PoolTest, GetMarginOfErrorForShot0Degrees) {
  Vector2d ghost_ball = Vector2d(diamonds(4), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(0));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(0.12094904776482043, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, GetMarginOfErrorForShot30Degrees) {
  Vector2d ghost_ball = Vector2d(diamonds(4), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(30));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;
  EXPECT_DOUBLE_EQ(0.10627717171291465, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, GetMarginOfErrorForShot60Degrees) {
  Vector2d ghost_ball = Vector2d(diamonds(4), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(60));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(0.063049735936784909, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, GetMarginOfErrorForShot0Degrees2Diamonds) {
  Vector2d ghost_ball = Vector2d(diamonds(2), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(0));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(0.48281531942723194, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, GetMarginOfErrorForShot0Degrees1Diamond) {
  Vector2d ghost_ball = Vector2d(diamonds(1), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(0));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(1.9123518340796108, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, GetMarginOfErrorForShotTooClose) {
  Vector2d ghost_ball = Vector2d(diamonds(2), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = Vector2d(diamonds(0.2), 0);
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(0, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, GetMarginOfErrorForShotImpossibleCueBallAngle) {
  Vector2d ghost_ball = Vector2d(diamonds(4), 0);
  Rotation2Dd ghost_ball_rotation(degrees_to_radians(45));
  ghost_ball = ghost_ball_rotation * ghost_ball;
  Vector2d cue_ball = ghost_ball;
  Rotation2Dd cue_ball_rotation(degrees_to_radians(-95));
  cue_ball = cue_ball_rotation * cue_ball;
  cue_ball += ghost_ball;

  EXPECT_DOUBLE_EQ(0, radians_to_degrees(get_margin_of_error_for_shot(cue_ball, ghost_ball, pockets[0], Speed::FAST)));
}

TEST_F(PoolTest, DeciangleToEffectivePocketSize)
{
  EXPECT_DOUBLE_EQ(0.165175899265596, deciangle_to_effective_pocket_size(429, deciangle_to_effective_pocket_size_slow_side));
}

TEST_F(PoolTest, Precomputed)
{
  EXPECT_DOUBLE_EQ(0.165175899265596, deciangle_to_effective_pocket_size_slow_side[429]);
  EXPECT_DOUBLE_EQ(0.012668196463007301, deciangle_to_effective_pocket_size_fast_side[497]);
  EXPECT_DOUBLE_EQ(0.18710080701590601, deciangle_to_effective_pocket_size_fast_corner[51]);
  EXPECT_DOUBLE_EQ(0.18966694044239801, deciangle_to_effective_pocket_size_slow_corner[308]);
  for (short i = 0; i <= 667; ++i) {
    EXPECT_TRUE(deciangle_to_effective_pocket_size_slow_side.find(i) != deciangle_to_effective_pocket_size_slow_side.end());
  }
  for (short i = 0; i <= 499; ++i) {
    EXPECT_TRUE(deciangle_to_effective_pocket_size_fast_side.find(i) != deciangle_to_effective_pocket_size_fast_side.end());
  }
  for (short i = 0; i <= 450; ++i) {
    EXPECT_TRUE(deciangle_to_effective_pocket_size_fast_corner.find(i) != deciangle_to_effective_pocket_size_fast_side.end());
  }
  for (short i = 0; i <= 449; ++i) {
    EXPECT_TRUE(deciangle_to_effective_pocket_size_slow_corner.find(i) != deciangle_to_effective_pocket_size_fast_side.end());
  }
}

TEST_F(PoolTest, PopulateShotTableDifficulty)
{
  initialize(
    Ball(0, 4), /* Cue ball */
    Ball(0, 4), /* Eight ball */
    {Ball(2, 2), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4)}, /* Player balls */
    {Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4), Ball(0, 4)} /* Opponent balls */
  );
  populate_player_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_table_obstructions();
  populate_shot_table_difficulty();
  EXPECT_VEC2D_EQ(Vector2d(8.5091168824543146, 8.5091168824543146), shot_table[8][12][1][0].get_ghost_ball()->get_coords());
  EXPECT_VEC2D_EQ(Vector2d(0.5091168824543146, -3.4908831175456854), shot_table[8][12][1][0].get_cue_ball_to_ghost_ball());
  EXPECT_DOUBLE_EQ(109.48156660347786, shot_table[8][12][1][0].get_slow_shot_difficulty());
  EXPECT_DOUBLE_EQ(124.95342530513796, shot_table[8][12][1][0].get_fast_shot_difficulty());
  EXPECT_DOUBLE_EQ(11986.213425951759, shot_table[8][12][1][0].get_weighted_slow_shot_difficulty());
  EXPECT_DOUBLE_EQ(15613.358495486691, shot_table[8][12][1][0].get_weighted_fast_shot_difficulty());
  EXPECT_TRUE(shot_table[8][12][1][0].get_possible());
  EXPECT_DOUBLE_EQ(-0.64569555916479759, shot_table[9][12][1][0].get_shot_angle());
  EXPECT_FALSE(shot_table[0][6][1][0].get_possible());
}

TEST_F(PoolTest, MoveBallInFromRails)
{
  EXPECT_VEC2D_EQ(Vector2d(diamonds(4), BALL_RADIUS), move_ball_in_from_rails(Vector2d(diamonds(4), diamonds(0))));
  EXPECT_VEC2D_EQ(Vector2d(diamonds(3), WIDTH - BALL_RADIUS), move_ball_in_from_rails(Vector2d(diamonds(3), diamonds(7.9))));
  EXPECT_VEC2D_EQ(Vector2d(BALL_RADIUS, diamonds(1)), move_ball_in_from_rails(Vector2d(diamonds(0), diamonds(1))));
  EXPECT_VEC2D_EQ(Vector2d(LENGTH - BALL_RADIUS, diamonds(3)), move_ball_in_from_rails(Vector2d(diamonds(7.98), diamonds(3))));
}

TEST_F(PoolTest, PopulateShotTableObstructions)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(6, 1), /* Eight ball */
    {Ball(4, 2), Ball(7, 0.5), Ball(4, 1), Ball(to_diamonds(BALL_RADIUS), 2), Ball(3, 2)}, /* Player balls */
    {Ball(4, 3), Ball(2, 3)}); /* Opponent balls */
  populate_player_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_table_obstructions();
  EXPECT_FALSE(shot_table[7][13][0][5].get_possible()); // Cue ball blocked by opponent ball.
  EXPECT_EQ(0, shot_table[12][12][0][5].get_shot_obstructions().get_obstructing_player_balls().size()); // Eight ball is never blocked by player balls.
  EXPECT_THAT(shot_table[8][0][1][5].get_shot_obstructions().get_obstructing_player_balls(), ElementsAre()); // No obstructions.
  EXPECT_THAT(shot_table[8][0][1][1].get_shot_obstructions().get_obstructing_player_balls(), ElementsAre(3)); // Player ball blocked by player ball.
  EXPECT_THAT(shot_table[8][8][1][5].get_shot_obstructions().get_obstructing_player_balls(), ElementsAre(5)); // Cue ball blocked by player ball.
  EXPECT_THAT(shot_table[8][8][1][1].get_shot_obstructions().get_obstructing_player_balls(), ElementsAre(3, 5)); // Cue ball blocked by player ball. Player ball blocked by player ball.
  EXPECT_TRUE(shot_table[28][8][3][2].get_possible()); // No obstructions.
  EXPECT_EQ(0, shot_table[28][8][0][2].get_shot_obstructions().get_obstructing_player_balls().size()); // No obstructions.
  EXPECT_TRUE(shot_table[0][0][4][0].get_possible()); // Ghost ball valid.
  EXPECT_FALSE(shot_table[0][0][4][1].get_possible()); // Ghost ball invalid.
  EXPECT_FALSE(shot_table[0][0][4][2].get_possible()); // Ghost ball invalid.
  EXPECT_TRUE(shot_table[0][0][4][3].get_possible()); // Ghost ball valid.
  EXPECT_FALSE(shot_table[0][0][4][4].get_possible()); // Ghost ball invalid.
  EXPECT_FALSE(shot_table[0][0][4][5].get_possible()); // Ghost ball invalid.
}

TEST_F(PoolTest, InsertIntoSet)
{
  set<short> set1;
  set1.insert(1);
  set1.insert(2);
  set1.insert(3);
  set<short> set2;
  set2.insert(4);
  set2.insert(5);
  set2.insert(6);
  insert_into_set(set1, set2);
  EXPECT_EQ(6, set1.size());
  EXPECT_EQ(3, set2.size());
}

TEST_F(PoolTest, GetGhostBallForShot)
{
  EXPECT_VEC2D_EQ(Vector2d(16.64398757751994, 8.3219937887599702), get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[0]).get_coords());
  EXPECT_VEC2D_EQ(Vector2d(16, 8.7200000000000006), get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[1]).get_coords());
  EXPECT_VEC2D_EQ(Vector2d(15.356012422480061, 8.3219937887599702), get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[2]).get_coords());
  EXPECT_VEC2D_EQ(Vector2d(16.64398757751994, 7.6780062112400307), get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[3]).get_coords());
  EXPECT_VEC2D_EQ(Vector2d(16, 7.2800000000000002), get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[4]).get_coords());
  EXPECT_VEC2D_EQ(Vector2d(15.356012422480061, 7.6780062112400307), get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[5]).get_coords());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[0]).get_possible());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[1]).get_possible());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[2]).get_possible());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[3]).get_possible());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[4]).get_possible());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(diamonds(4), diamonds(2)), pockets[5]).get_possible());
}

TEST_F(PoolTest, GetGhostBallForImpossibleShot)
{
  EXPECT_VEC2D_EQ(Vector2d(0.50133172986702967, 8.7188126933396486), get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[0]).get_coords());
  EXPECT_VEC2D_EQ(Vector2d(0.50133172986702967, 7.2811873066603523), get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[3]).get_coords());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[0]).get_possible());
  EXPECT_TRUE(get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[3]).get_possible());
  EXPECT_FALSE(get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[1]).get_possible());
  EXPECT_FALSE(get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[2]).get_possible());
  EXPECT_FALSE(get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[4]).get_possible());
  EXPECT_FALSE(get_ghost_ball_for_shot(Vector2d(BALL_RADIUS + 0.1, diamonds(2)), pockets[5]).get_possible());
}

TEST_F(PoolTest, PopulateGhostBallPositionTable)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(4, 2), /* Eight ball */
    {Ball(4, 2), Ball(4, 2)}, /* Player balls */
    {} /* Opponent balls */
  );
  populate_ghost_ball_position_table();
  for (short i = 0; i < player_balls.size(); ++i)
  {
    EXPECT_VEC2D_EQ(Vector2d(16.64398757751994, 8.3219937887599702), ghost_ball_position_table[i][0].get_coords());
    EXPECT_VEC2D_EQ(Vector2d(16, 8.7200000000000006), ghost_ball_position_table[i][1].get_coords());
    EXPECT_VEC2D_EQ(Vector2d(15.356012422480061, 8.3219937887599702), ghost_ball_position_table[i][2].get_coords());
    EXPECT_VEC2D_EQ(Vector2d(16.64398757751994, 7.6780062112400307), ghost_ball_position_table[i][3].get_coords());
    EXPECT_VEC2D_EQ(Vector2d(16, 7.2800000000000002), ghost_ball_position_table[i][4].get_coords());
    EXPECT_VEC2D_EQ(Vector2d(15.356012422480061, 7.6780062112400307), ghost_ball_position_table[i][5].get_coords());
  }
}

TEST_F(PoolTest, DistanceFromPointToSegment_aboveSegment)
{
  EXPECT_DOUBLE_EQ(3, distance_from_point_to_segment(Vector2d(2, 3), Vector2d(-5, 0), Vector2d(4, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_aboveSegmentReversed)
{
  EXPECT_DOUBLE_EQ(3, distance_from_point_to_segment(Vector2d(2, 3), Vector2d(4, 0), Vector2d(-5, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_belowSegment)
{
  EXPECT_DOUBLE_EQ(3, distance_from_point_to_segment(Vector2d(2, -3), Vector2d(4, 0), Vector2d(-5, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_rightOfSegment)
{
  EXPECT_DOUBLE_EQ(numeric_limits<double>::infinity(), distance_from_point_to_segment(
    Vector2d(2, 3), Vector2d(-5, 0), Vector2d(1, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_leftOfSegment)
{
  EXPECT_DOUBLE_EQ(numeric_limits<double>::infinity(), distance_from_point_to_segment(
    Vector2d(2, 3), Vector2d(4, 0), Vector2d(8, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_angledLine)
{
  EXPECT_DOUBLE_EQ(1.4142135623730951, distance_from_point_to_segment(
    Vector2d(0, 2), Vector2d(1, 1), Vector2d(5, 5)));
  EXPECT_DOUBLE_EQ(1.4142135623730951, distance_from_point_to_segment(
    Vector2d(0, 2), Vector2d(-1, 1), Vector2d(-5, 5)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_onSegment)
{
  EXPECT_DOUBLE_EQ(0, distance_from_point_to_segment(Vector2d(4, 0), Vector2d(4, 0), Vector2d(8, 0)));
}

TEST_F(PoolTest, GetSegmentRanges)
{
  SegmentRange segment_ranges = get_segment_range(Vector2d(-1, 5), Vector2d(-3, 3));
  EXPECT_EQ(-3, segment_ranges.min_x);
  EXPECT_EQ(-1, segment_ranges.max_x);
  EXPECT_EQ(3, segment_ranges.min_y);
  EXPECT_EQ(5, segment_ranges.max_y);
}

TEST_F(PoolTest, GetBallIntersectsSegment_intersectsMiddle)
{
  EXPECT_TRUE(get_ball_intersects_ball_path(
    Vector2d(diamonds(2), diamonds(4)),
    Vector2d(0, diamonds(4)),
    Vector2d(diamonds(4), diamonds(4))));
}

TEST_F(PoolTest, GetBallIntersectsSegment_intersectsTop)
{
  EXPECT_TRUE(get_ball_intersects_ball_path(
    Vector2d(diamonds(2), diamonds(4) + BALL_DIAMETER - 0.001),
    Vector2d(0, diamonds(4)),
    Vector2d(diamonds(4), diamonds(4))));
}

TEST_F(PoolTest, GetBallIntersectsSegment_intersectsBottom)
{
  EXPECT_TRUE(get_ball_intersects_ball_path(
    Vector2d(diamonds(2), diamonds(4) - BALL_DIAMETER + 0.001),
    Vector2d(0, diamonds(4)),
    Vector2d(diamonds(4), diamonds(4))));
}

TEST_F(PoolTest, GetBallIntersectsSegment_ballAbove)
{
  EXPECT_FALSE(get_ball_intersects_ball_path(
    Vector2d(diamonds(2), diamonds(4) + BALL_DIAMETER + 0.001),
    Vector2d(0, diamonds(4)),
    Vector2d(diamonds(4), diamonds(4))));
}

TEST_F(PoolTest, GetBallIntersectsSegment_ballBelow)
{
  EXPECT_FALSE(get_ball_intersects_ball_path(
    Vector2d(diamonds(2), diamonds(4) - BALL_DIAMETER - 0.001),
    Vector2d(0, diamonds(4)),
    Vector2d(diamonds(4), diamonds(4))));
}

TEST_F(PoolTest, GetBallIntersectsSegment_angledLine)
{
  EXPECT_TRUE(get_ball_intersects_ball_path(
    Vector2d(diamonds(1), diamonds(1)),
    Vector2d(diamonds(4 / 3), diamonds(4 / 3)),
    Vector2d(0, 0)));
}

TEST_F(PoolTest, GetBallIntersectsSegment_angledLineMiss)
{
  EXPECT_FALSE(get_ball_intersects_ball_path(
    Vector2d(diamonds(1), diamonds(1.4)),
    Vector2d(diamonds(2), diamonds(2)),
    Vector2d(0, 0)));
}

TEST_F(PoolTest, BallIntersectsSegment_fromCornerToMiddle)
{
  EXPECT_TRUE(get_ball_intersects_ball_path(
    Vector2d(diamonds(1), diamonds(6)),
    Vector2d(0, diamonds(8)),
    Vector2d(diamonds(2), diamonds(4))));
}

TEST_F(PoolTest, EightBallIndex) {
  EXPECT_EQ(0, eight_ball_index());
}

TEST_F(PoolTest, GetObstructionsOnBallPathForBallIndex_unobstructed)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(3, 2), /* Eight ball */
    {Ball(1, 1), Ball(3, 1)}, /* Player balls */
    {Ball(4, 1)}); /* Opponent balls */
  BallObstructions shot_obstructions = get_obstructions_on_ball_path_for_ball_index(
    1,
    Vector2d(diamonds(1), diamonds(1)),
    Vector2d(diamonds(4), diamonds(4)),
    false);
  EXPECT_FALSE(shot_obstructions.get_has_permanent_obstruction());
  EXPECT_EQ(0, shot_obstructions.get_obstructing_player_balls().size());
}

TEST_F(PoolTest, GetObstructionsOnBallPathForBallIndex_eight_ball_obstruction)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(3, 3), /* Eight ball */
    {Ball(1, 1), Ball(3, 1)}, /* Player balls */
    {Ball(4, 1)}); /* Opponent balls */
  BallObstructions shot_obstructions = get_obstructions_on_ball_path_for_ball_index(
    1,
    Vector2d(diamonds(1), diamonds(1)),
    Vector2d(diamonds(4), diamonds(4)),
    false);
  EXPECT_TRUE(shot_obstructions.get_has_permanent_obstruction());
  EXPECT_EQ(0, shot_obstructions.get_obstructing_player_balls().size());
}

TEST_F(PoolTest, GetObstructionsOnBallPathForBallIndex_opponent_ball_obstruction)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(2, 1), /* Eight ball */
    {Ball(1, 1), Ball(3, 1)}, /* Player balls */
    {Ball(3, 3)}); /* Opponent balls */
  BallObstructions shot_obstructions = get_obstructions_on_ball_path_for_ball_index(
    1, Vector2d(diamonds(1), diamonds(1)), Vector2d(diamonds(4), diamonds(4)), false);
  EXPECT_TRUE(shot_obstructions.get_has_permanent_obstruction());
  EXPECT_EQ(0, shot_obstructions.get_obstructing_player_balls().size());
}

TEST_F(PoolTest, GetObstructionsOnBallPathForBallIndex_player_ball_obstruction)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(2, 1), /* Eight ball */
    {Ball(1, 1), Ball(3, 1), Ball(2, 2), Ball(3, 1), Ball(3, 1), Ball(2.5, 2.6), Ball(3, 1)}, /* Player balls */
    {Ball(4, 1)}); /* Opponent balls */
  BallObstructions shot_obstructions = get_obstructions_on_ball_path_for_ball_index(
    1, Vector2d(diamonds(1), diamonds(1)), Vector2d(diamonds(4), diamonds(4)), false);
  EXPECT_FALSE(shot_obstructions.get_has_permanent_obstruction());
  ASSERT_THAT(shot_obstructions.get_obstructing_player_balls(), ElementsAre(3, 6));
}

TEST_F(PoolTest, GetObstructionsOnBallPathForBallIndex_pocket_obstruction)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(2, 1), /* Eight ball */
    {Ball(1, 1)}, /* Player balls */
    {}); /* Opponent balls */
  eight_ball = Vector2d(UNITS_PER_DIAMOND, UNITS_PER_DIAMOND * 2);
  player_balls.push_back(Vector2d(diamonds(1), diamonds(1)));
  BallObstructions shot_obstructions = get_obstructions_on_ball_path_for_ball_index(
    1, Vector2d(diamonds(1), diamonds(1)), Vector2d(diamonds(5), diamonds(5)), true);
  EXPECT_TRUE(shot_obstructions.get_has_permanent_obstruction());
  EXPECT_EQ(0, shot_obstructions.get_obstructing_player_balls().size());
}

TEST_F(PoolTest, GetObstructionsOnBallPathForBallIndex_eight_ball_opponent_ball_and_pocket_obstruction)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(1, 1), /* Eight ball */
    {Ball(2, 2)}, /* Player balls */
    {Ball(3, 3)}); /* Opponent balls */
  BallObstructions shot_obstructions = get_obstructions_on_ball_path_for_ball_index(
    0, Vector2d(diamonds(1), diamonds(1)), Vector2d(diamonds(5), diamonds(5)), true);
  EXPECT_FALSE(shot_obstructions.get_has_permanent_obstruction());
  EXPECT_EQ(0, shot_obstructions.get_obstructing_player_balls().size());
}

TEST_F(PoolTest, PopulateBallToPocketObstructionsTable)
{
  initialize(
    Ball(4, 2), /* Cue ball */
    Ball(6, 1), /* Eight ball */
    {Ball(4, 2), Ball(7, 0.5), Ball(4, 1)}, /* Player balls */
    {Ball(4, 3), Ball(2, 3)}); /* Opponent balls */
  populate_player_ball_to_pocket_obstructions_table();
  for (short p = 0; p < 6; ++p)
  {
    for (short b = 0; b < player_balls.size(); ++b)
    {
      if (b == 0) {
        if (p == 3)
        {
          EXPECT_TRUE(player_ball_to_pocket_obstructions_table[b][p].get_has_permanent_obstruction());
        }
        else
        {
          EXPECT_FALSE(player_ball_to_pocket_obstructions_table[b][p].get_has_permanent_obstruction());
        }
        continue;
      }
      if ((b == 1 && p == 2) || (b == 1 && p == 3) || (b == 1 && p == 4) ||
          (b == 2 && p == 3) || (b == 3 && p == 4))
      {
        EXPECT_TRUE(player_ball_to_pocket_obstructions_table[b][p].get_has_permanent_obstruction());
      }
      else
      {
        EXPECT_FALSE(player_ball_to_pocket_obstructions_table[b][p].get_has_permanent_obstruction());
      }
      if ((b == 1 && p == 1))
      {
        EXPECT_THAT(player_ball_to_pocket_obstructions_table[b][p].get_obstructing_player_balls(), ElementsAre(3));
      }
      else
      {
        EXPECT_EQ(0, player_ball_to_pocket_obstructions_table[b][p].get_obstructing_player_balls().size());
      }
    }
  }
}

TEST_F(PoolTest, TableEdgesInitialized)
{
  EXPECT_VEC2D_EQ((pockets[0].get_position() + Vector2d(BALL_RADIUS, BALL_RADIUS)), bottom_edge[0]);
  EXPECT_VEC2D_EQ((pockets[2].get_position() + Vector2d(-1 * BALL_RADIUS, BALL_RADIUS)), bottom_edge[1]);
  EXPECT_VEC2D_EQ((pockets[2].get_position() + Vector2d(-1 * BALL_RADIUS, BALL_RADIUS)), right_edge[0]);
  EXPECT_VEC2D_EQ((pockets[5].get_position() + Vector2d(-1 * BALL_RADIUS, -1 * BALL_RADIUS)), right_edge[1]);
  EXPECT_VEC2D_EQ((pockets[5].get_position() + Vector2d(-1 * BALL_RADIUS, -1 * BALL_RADIUS)), top_edge[0]);
  EXPECT_VEC2D_EQ((pockets[3].get_position() + Vector2d(BALL_RADIUS, -1 * BALL_RADIUS)), top_edge[1]);
  EXPECT_VEC2D_EQ((pockets[3].get_position() + Vector2d(BALL_RADIUS, -1 * BALL_RADIUS)), left_edge[0]);
  EXPECT_VEC2D_EQ((pockets[0].get_position() + Vector2d(BALL_RADIUS, BALL_RADIUS)), left_edge[1]);
}

TEST_F(PoolTest, PocketsInitialized)
{
  EXPECT_EQ(6, pockets.size());
  EXPECT_EQ(Vector2d(0, 0), pockets[0].get_position());
  EXPECT_EQ(Vector2d(WIDTH, 0), pockets[1].get_position());
  EXPECT_EQ(Vector2d(LENGTH, 0), pockets[2].get_position());
  EXPECT_EQ(Vector2d(0, WIDTH), pockets[3].get_position());
  EXPECT_EQ(Vector2d(WIDTH, WIDTH), pockets[4].get_position());
  EXPECT_EQ(Vector2d(LENGTH, WIDTH), pockets[5].get_position());
  EXPECT_EQ(Vector2d(1, 1), pockets[0].get_center_line());
  EXPECT_EQ(Vector2d(0, 1), pockets[1].get_center_line());
  EXPECT_EQ(Vector2d(-1, 1), pockets[2].get_center_line());
  EXPECT_EQ(Vector2d(1, -1), pockets[3].get_center_line());
  EXPECT_EQ(Vector2d(0, -1), pockets[4].get_center_line());
  EXPECT_EQ(Vector2d(-1, -1), pockets[5].get_center_line());
}

} // namespace

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}