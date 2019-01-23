#include "./pool.h"
#include "gtest/gtest.h"
#include <iterator> //for std::ostream_iterator
#include <algorithm> //for std::copy

namespace {

// The fixture for testing class Pool.
class PoolTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  PoolTest() {
     // You can do set-up work for each test here.
  }

  ~PoolTest() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
    pockets.clear();
    object_balls.clear();
    opponent_object_balls.clear();
    eight_ball = Vector2d(0, 0);
    cue_ball = Vector2d(0, 0);
    ball_to_pocket_obstructions_table.clear();
    ghost_ball_position_table.clear();
    shot_info_table.clear();
    shot_path_table.clear();
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Pool.
};

void expect_vector2d_equal(Vector2d vec1, Vector2d vec2) {
  EXPECT_FLOAT_EQ(vec1.x(), vec2.x());
  EXPECT_FLOAT_EQ(vec1.y(), vec2.y());
}

TEST_F(PoolTest, InitializePockets) {
  initialize_pockets();
  EXPECT_EQ(6, pockets.size());
  EXPECT_EQ(Vector2d(0,0), pockets[0]);
  EXPECT_EQ(Vector2d(WIDTH,0), pockets[1]);
  EXPECT_EQ(Vector2d(0,WIDTH), pockets[2]);
  EXPECT_EQ(Vector2d(WIDTH, WIDTH), pockets[3]);
  EXPECT_EQ(Vector2d(0, LENGTH), pockets[4]);
  EXPECT_EQ(Vector2d(WIDTH, LENGTH), pockets[5]);
}

TEST_F(PoolTest, InitializeTableEdges) {
  initialize_pockets();
  initialize_table_edges();
  expect_vector2d_equal(pockets[0], bottom_edge[0]);
  expect_vector2d_equal(pockets[1], bottom_edge[1]);
  expect_vector2d_equal(pockets[1], right_edge[0]);
  expect_vector2d_equal(pockets[5], right_edge[1]);
  expect_vector2d_equal(pockets[5], top_edge[0]);
  expect_vector2d_equal(pockets[4], top_edge[1]);
  expect_vector2d_equal(pockets[4], left_edge[0]);
  expect_vector2d_equal(pockets[0], left_edge[1]);
}

TEST_F(PoolTest, DistanceFromPointToSegment_withinSegment) {
  EXPECT_FLOAT_EQ(3, distance_from_point_to_segment(Vector2d(2, 3), Vector2d(-5, 0), Vector2d(4, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_withinSegmentInverted) {
  EXPECT_FLOAT_EQ(3, distance_from_point_to_segment(Vector2d(2, 3), Vector2d(4, 0), Vector2d(-5, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_rightOfSegment) {
  EXPECT_FLOAT_EQ(-1, distance_from_point_to_segment(Vector2d(2, 3), Vector2d(-5, 0), Vector2d(1, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_leftOfSegment) {
  EXPECT_FLOAT_EQ(-1, distance_from_point_to_segment(Vector2d(2, 3), Vector2d(4, 0), Vector2d(8, 0)));
}

TEST_F(PoolTest, DistanceFromPointToSegment_angledLine) {
  EXPECT_FLOAT_EQ(1.4142135, distance_from_point_to_segment(Vector2d(0, 2), Vector2d(1, 1), Vector2d(5, 5)));
  EXPECT_FLOAT_EQ(1.4142135, distance_from_point_to_segment(Vector2d(0, 2), Vector2d(-1, 1), Vector2d(-5, 5)));
}

TEST_F(PoolTest, BallIntersectsSegment_intersectsMiddle) {
  EXPECT_TRUE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), Vector2d(0, DIAMOND_LENGTH * 4), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4)));
}

TEST_F(PoolTest, BallIntersectsSegment_intersectsTop) {
  EXPECT_TRUE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4 + BALL_DIAMETER - 0.001), Vector2d(0, DIAMOND_LENGTH * 4), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4)));
}

TEST_F(PoolTest, BallIntersectsSegment_intersectsBottom) {
  EXPECT_TRUE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4 - BALL_DIAMETER + 0.001), Vector2d(0, DIAMOND_LENGTH * 4), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4)));
}

TEST_F(PoolTest, BallIntersectsSegment_ballAbove) {
  EXPECT_FALSE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4 + BALL_DIAMETER + 0.001), Vector2d(0, DIAMOND_LENGTH * 4), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4)));
}

TEST_F(PoolTest, BallIntersectsSegment_ballBelow) {
  EXPECT_FALSE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4 - BALL_DIAMETER - 0.001), Vector2d(0, DIAMOND_LENGTH * 4), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4)));
}

TEST_F(PoolTest, BallIntersectsSegment_angledLine) {
  EXPECT_TRUE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH), Vector2d(DIAMOND_LENGTH * 4/3, DIAMOND_LENGTH * 4/3), Vector2d(0, 0)));
}

TEST_F(PoolTest, BallIntersectsSegment_angledLineMiss) {
  EXPECT_FALSE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH + DIAMOND_LENGTH / 2), Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 2), Vector2d(0, 0)));
}

TEST_F(PoolTest, BallIntersectsSegment_fromCornerToMiddle) {
  EXPECT_TRUE(ball_intersects_segment(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 6), Vector2d(0, DIAMOND_LENGTH * 8), Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4)));
}

TEST_F(PoolTest, GetObstructionsOnSegmentForShot_unobstructed) {
  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 2);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 4));
  obstructions shot_obstructions = get_obstructions_on_segment_for_shot(-1, Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4));
  EXPECT_FALSE(shot_obstructions.has_permanent_obstruction);
  EXPECT_EQ(0, shot_obstructions.obstructing_object_balls.size());
}

TEST_F(PoolTest, GetObstructionsOnSegmentForShot_eight_ball_obstruction) {
  eight_ball = Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 3);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 4));
  obstructions shot_obstructions = get_obstructions_on_segment_for_shot(-1, Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4));
  EXPECT_TRUE(shot_obstructions.has_permanent_obstruction);
  EXPECT_EQ(0, shot_obstructions.obstructing_object_balls.size());
}

TEST_F(PoolTest, GetObstructionsOnSegmentForShot_opponent_ball_obstruction) {
  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 2);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 3));
  obstructions shot_obstructions = get_obstructions_on_segment_for_shot(-1, Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4));
  EXPECT_TRUE(shot_obstructions.has_permanent_obstruction);
  EXPECT_EQ(0, shot_obstructions.obstructing_object_balls.size());
}

TEST_F(PoolTest, GetObstructionsOnSegmentForShot_object_ball_obstruction) {
  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 2);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 2)); // Obstructing.
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2.5 + DIAMOND_LENGTH / 4, DIAMOND_LENGTH * 2.5)); // Obstructing.
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 4));
  obstructions shot_obstructions = get_obstructions_on_segment_for_shot(-1, Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH), Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4));
  EXPECT_FALSE(shot_obstructions.has_permanent_obstruction);
  EXPECT_EQ(2, shot_obstructions.obstructing_object_balls.size());
  EXPECT_TRUE(shot_obstructions.obstructing_object_balls.find(1) != shot_obstructions.obstructing_object_balls.end());
  EXPECT_TRUE(shot_obstructions.obstructing_object_balls.find(4) != shot_obstructions.obstructing_object_balls.end());
}

TEST_F(PoolTest, PopulateBallToPocketObstructionsTable) {
  initialize_pockets();
  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 6);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 0.5, DIAMOND_LENGTH * 7));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 4));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 4));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 2));
  populate_ball_to_pocket_obstructions_table();
  for (unsigned char p = 0; p < 6; ++p) {
    for (unsigned char b = 0; b < 3; ++b) {
      if ((b == 0 && p == 1) || (b == 0 && p == 3) || (b == 0 && p == 4) ||
          (b == 1 && p == 1) || (b == 2 && p == 3)) {
        EXPECT_TRUE(ball_to_pocket_obstructions_table[b][p].has_permanent_obstruction);
      } else {
        EXPECT_FALSE(ball_to_pocket_obstructions_table[b][p].has_permanent_obstruction);
      }
      if ((b == 0 && p == 2)) {
        EXPECT_EQ(1, ball_to_pocket_obstructions_table[b][p].obstructing_object_balls.size());
        EXPECT_TRUE(ball_to_pocket_obstructions_table[b][p].obstructing_object_balls.find(2) != ball_to_pocket_obstructions_table[b][p].obstructing_object_balls.end());
      } else {
        EXPECT_EQ(0, ball_to_pocket_obstructions_table[b][p].obstructing_object_balls.size());
      }
    }
    if (p == 1) {
      EXPECT_TRUE(ball_to_pocket_obstructions_table[object_balls.size()][p].has_permanent_obstruction);
    } else {
      EXPECT_FALSE(ball_to_pocket_obstructions_table[object_balls.size()][p].has_permanent_obstruction);
    }
    if (p == 4) {
      EXPECT_EQ(1, ball_to_pocket_obstructions_table[object_balls.size()][p].obstructing_object_balls.size());
      EXPECT_TRUE(ball_to_pocket_obstructions_table[object_balls.size()][p].obstructing_object_balls.find(1) != ball_to_pocket_obstructions_table[object_balls.size()][p].obstructing_object_balls.end());
    } else {
      EXPECT_EQ(0, ball_to_pocket_obstructions_table[object_balls.size()][p].obstructing_object_balls.size());
    }
  }
}

TEST_F(PoolTest, GetGhostBallForShot) {
  initialize_pockets();
  expect_vector2d_equal(Vector2d(4.1609969, 8.3219938), get_ghost_ball_for_shot(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), pockets[0]));
  expect_vector2d_equal(Vector2d(3.8390031, 8.3219938), get_ghost_ball_for_shot(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), pockets[1]));
  expect_vector2d_equal(Vector2d(4.3600001, 8), get_ghost_ball_for_shot(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), pockets[2]));
  expect_vector2d_equal(Vector2d(3.6399999, 8), get_ghost_ball_for_shot(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), pockets[3]));
  expect_vector2d_equal(Vector2d(4.1609969, 7.6780062), get_ghost_ball_for_shot(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), pockets[4]));
  expect_vector2d_equal(Vector2d(3.8390031, 7.6780062), get_ghost_ball_for_shot(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4), pockets[5]));
}

TEST_F(PoolTest, PopulateGhostBallPositionTable) {
  initialize_pockets();
  eight_ball = Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4));
  populate_ghost_ball_position_table();
  for (unsigned char i = 0; i < 8; ++i) {
    if (i >= 2 && i <= 7) {
      continue;
    }
    expect_vector2d_equal(Vector2d(4.1609969, 8.3219938), ghost_ball_position_table[i][0]);
    expect_vector2d_equal(Vector2d(3.8390031, 8.3219938), ghost_ball_position_table[i][1]);
    expect_vector2d_equal(Vector2d(4.3600001, 8), ghost_ball_position_table[i][2]);
    expect_vector2d_equal(Vector2d(3.6399999, 8), ghost_ball_position_table[i][3]);
    expect_vector2d_equal(Vector2d(4.1609969, 7.6780062), ghost_ball_position_table[i][4]);
    expect_vector2d_equal(Vector2d(3.8390031, 7.6780062), ghost_ball_position_table[i][5]);
  }
}

TEST_F(PoolTest, InsertIntoSet) {
  set<unsigned char> set1;
  set1.insert(1);
  set1.insert(2);
  set1.insert(3);
  set<unsigned char> set2;
  set2.insert(4);
  set2.insert(5);
  set2.insert(6);
  insert_into_set(set1, set2);
  EXPECT_EQ(6, set1.size());
  EXPECT_EQ(3, set2.size());
}

TEST_F(PoolTest, GetIntersectionOfLineSegments) {
  segment_intersection_struct segment_intersection = get_intersection_of_line_segments(
      Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 4),
      Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 4),
      Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3),
      Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 5));
  EXPECT_TRUE(segment_intersection.has_intersection);
  expect_vector2d_equal(Vector2d(3, 8), segment_intersection.intersection_point);
}

TEST_F(PoolTest, GetIntersectionOfLineSegments_noIntersection) {
  segment_intersection_struct segment_intersection = get_intersection_of_line_segments(
      Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH),
      Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3),
      Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH),
      Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 3));
  EXPECT_FALSE(segment_intersection.has_intersection);
}

TEST_F(PoolTest, GetIntersectionOfLineSegments_notOnSegments) {
  segment_intersection_struct segment_intersection = get_intersection_of_line_segments(
      Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH),
      Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3),
      Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 2),
      Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 2));
  EXPECT_FALSE(segment_intersection.has_intersection);
}

TEST_F(PoolTest, GetIntersectionWithTableEdges) {
  initialize_pockets();
  initialize_table_edges();
  segment_intersection_struct segment_intersection = get_intersection_of_line_segments(
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 7),
    Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 9),
    top_edge);
  EXPECT_TRUE(segment_intersection.has_intersection);
  expect_vector2d_equal(Vector2d(5, 16), segment_intersection.intersection_point);
}

TEST_F(PoolTest, GetSegmentRanges) {
  segment_range_struct segment_ranges = get_segment_ranges(Vector2d(-1, 5), Vector2d(-3, 3));
  EXPECT_EQ(-3, segment_ranges.min_x);
  EXPECT_EQ(-1, segment_ranges.max_x);
  EXPECT_EQ(3, segment_ranges.min_y);
  EXPECT_EQ(5, segment_ranges.max_y);
}

TEST_F(PoolTest, GetTangentLine_perpendicularToRail) {
  shot_angle_struct shot_angle = get_shot_angle(
    Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 3),
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4),
    Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4));
  EXPECT_EQ(shot_angle_struct::CLOCKWISE, shot_angle.follow_direction);
  expect_vector2d_equal(Vector2d(0, 1), shot_angle.tangent_line_direction);
  expect_vector2d_equal(Vector2d(4, 8), shot_angle.origin);
  expect_vector2d_equal(Vector2d(1, 0), shot_angle.pocket_direction);
  EXPECT_FLOAT_EQ(0.5, shot_angle.fractional_distance);
}

TEST_F(PoolTest, GetTangentLine_perpendicularToRailOppositeDirection) {
  shot_angle_struct shot_angle = get_shot_angle(
    Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 5),
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4),
    Vector2d(DIAMOND_LENGTH * 4, DIAMOND_LENGTH * 4));
  EXPECT_EQ(shot_angle_struct::COUNTER_CLOCKWISE, shot_angle.follow_direction);
  expect_vector2d_equal(Vector2d(0, -1), shot_angle.tangent_line_direction);
  expect_vector2d_equal(Vector2d(4, 8), shot_angle.origin);
  expect_vector2d_equal(Vector2d(1, 0), shot_angle.pocket_direction);
  EXPECT_FLOAT_EQ(0.5, shot_angle.fractional_distance);
}

TEST_F(PoolTest, GetTangentLine_45Degrees) {
  shot_angle_struct shot_angle = get_shot_angle(
    Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH),
    Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH),
    Vector2d(0, 0));
  EXPECT_EQ(shot_angle_struct::COUNTER_CLOCKWISE, shot_angle.follow_direction);
  expect_vector2d_equal(Vector2d(-0.70710677, 0.70710677), shot_angle.tangent_line_direction);
  expect_vector2d_equal(Vector2d(2, 2), shot_angle.origin);
  expect_vector2d_equal(Vector2d(-0.70710677, -0.70710677), shot_angle.pocket_direction);
  EXPECT_FLOAT_EQ(0.5, shot_angle.fractional_distance);
}

TEST_F(PoolTest, ReflectBallPathOffTableEdges_rightEdge) {
  initialize_pockets();
  initialize_table_edges();
  rail_reflection_struct rail_reflection = reflect_ball_path_off_table_edges(
    Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 6),
    Vector2d(DIAMOND_LENGTH * 6, DIAMOND_LENGTH * 9),
    8.48528137424);
    EXPECT_TRUE(rail_reflection.has_intersection);
    expect_vector2d_equal(Vector2d(8, 14), rail_reflection.intersection_point);
    expect_vector2d_equal(Vector2d(4, 18), rail_reflection.end_point);
}

TEST_F(PoolTest, ReflectBallPathOffTableEdges_topEdge) {
  initialize_pockets();
  initialize_table_edges();
  rail_reflection_struct rail_reflection = reflect_ball_path_off_table_edges(
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 7),
    Vector2d(DIAMOND_LENGTH * 5, DIAMOND_LENGTH * 10),
    8.48528137424);
    EXPECT_TRUE(rail_reflection.has_intersection);
    expect_vector2d_equal(Vector2d(6, 16), rail_reflection.intersection_point);
    expect_vector2d_equal(Vector2d(10, 12), rail_reflection.end_point);
}

TEST_F(PoolTest, ReflectBallPathOffTableEdges_bottomEdge) {
  initialize_pockets();
  initialize_table_edges();
  rail_reflection_struct rail_reflection = reflect_ball_path_off_table_edges(
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH),
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * -10),
    22);
    EXPECT_TRUE(rail_reflection.has_intersection);
    expect_vector2d_equal(Vector2d(4, 0), rail_reflection.intersection_point);
    expect_vector2d_equal(Vector2d(4, 20), rail_reflection.end_point);
}

TEST_F(PoolTest, ReflectBallPathOffTableEdges_leftEdge) {
  initialize_pockets();
  initialize_table_edges();
  rail_reflection_struct rail_reflection = reflect_ball_path_off_table_edges(
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH),
    Vector2d(DIAMOND_LENGTH * -1, DIAMOND_LENGTH * 5),
    10);
    EXPECT_TRUE(rail_reflection.has_intersection);
    expect_vector2d_equal(Vector2d(0, 7.3333335), rail_reflection.intersection_point);
    expect_vector2d_equal(Vector2d(2, 10), rail_reflection.end_point);
}

TEST_F(PoolTest, ReflectBallPathOffTableEdges_noIntersection) {
  initialize_pockets();
  initialize_table_edges();
  rail_reflection_struct rail_reflection = reflect_ball_path_off_table_edges(
    Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH),
    Vector2d(DIAMOND_LENGTH * 3.5, DIAMOND_LENGTH * 7.5),
    13.3416640641);
    EXPECT_FALSE(rail_reflection.has_intersection);
}

TEST_F(PoolTest, StrengthToDistance) {
  EXPECT_FLOAT_EQ(MAX_DIAMONDS * DIAMOND_LENGTH, strength_to_distance(NUM_STRENGTHS));
  EXPECT_FLOAT_EQ(1.0 * MAX_DIAMONDS * DIAMOND_LENGTH / NUM_STRENGTHS, strength_to_distance(1));
}

TEST_F(PoolTest, GetPath_parallelToLongRail) {
  initialize_pockets();
  initialize_table_edges();
  shot_angle_struct shot_angle;
  shot_angle.follow_direction = shot_angle_struct::CLOCKWISE;
  shot_angle.tangent_line_direction = Vector2d(0, 1);
  shot_angle.origin = Vector2d(DIAMOND_LENGTH, 0);
  shot_angle.pocket_direction = Vector2d(1, 0);
  shot_angle.fractional_distance = 1;
  vector<Vector2d> path = get_path(shot_angle, NUM_STRENGTHS, 0);
  EXPECT_EQ(5, path.size());
  expect_vector2d_equal(Vector2d(2, 0), path[0]);
  expect_vector2d_equal(Vector2d(2, 16), path[1]);
  expect_vector2d_equal(Vector2d(2, 0), path[2]);
  expect_vector2d_equal(Vector2d(2, 16), path[3]);
  expect_vector2d_equal(Vector2d(2, 4), path[4]);
}

TEST_F(PoolTest, GetPath_doesNotIntersectRail) {
  initialize_pockets();
  initialize_table_edges();
  shot_angle_struct shot_angle;
  shot_angle.follow_direction = shot_angle_struct::CLOCKWISE;
  shot_angle.tangent_line_direction = Vector2d(0.707, 0.707);
  shot_angle.origin = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH);
  shot_angle.pocket_direction = Vector2d(-0.707, -0.707);
  shot_angle.fractional_distance = 0.1;
  vector<Vector2d> path = get_path(shot_angle, NUM_STRENGTHS, 0);
  EXPECT_EQ(2, path.size());
  expect_vector2d_equal(Vector2d(2, 2), path[0]);
  expect_vector2d_equal(Vector2d(6.2420001, 6.2420001), path[1]);
}

TEST_F(PoolTest, GetPath_intersectsRailAt45Degrees) {
  initialize_pockets();
  initialize_table_edges();
  shot_angle_struct shot_angle;
  shot_angle.follow_direction = shot_angle_struct::CLOCKWISE;
  shot_angle.tangent_line_direction = Vector2d(0.70710678118, 0.70710678118);
  shot_angle.origin = Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH);
  shot_angle.pocket_direction = Vector2d(-0.70710678118, -0.70710678118);
  shot_angle.fractional_distance = 1;
  vector<Vector2d> path = get_path(shot_angle, NUM_STRENGTHS, 0);
  EXPECT_EQ(9, path.size());
  expect_vector2d_equal(Vector2d(4, 2), path[0]);
  expect_vector2d_equal(Vector2d(8, 6), path[1]);
  expect_vector2d_equal(Vector2d(0, 14), path[2]);
  expect_vector2d_equal(Vector2d(1.999999, 16), path[3]);
  expect_vector2d_equal(Vector2d(8, 10), path[4]);
  expect_vector2d_equal(Vector2d(0, 1.9999983), path[5]);
  expect_vector2d_equal(Vector2d(1.9999983, 0), path[6]);
  expect_vector2d_equal(Vector2d(8, 6), path[7]);
  expect_vector2d_equal(Vector2d(1.5735935, 12.426409), path[8]);
}

// TODO: Not a real test, but the data looks okay.
TEST_F(PoolTest, PopulateShotInfoTableObstructions) {
  initialize_pockets();
  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 6);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 4));
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_info_table_obstructions();
  for (unsigned char w = 0; w <= WIDTH; ++w) {
    for (unsigned char l = 0; l <= LENGTH; ++l) {
      for (unsigned char p = 0; p < 6; ++p) {
        for (unsigned char b = object_balls.size(); b < object_balls.size() + 1; ++b) {
          if (shot_info_table[w][l][b][p].shot_obstructions.obstructing_object_balls.size() > 0) {
            cout << (int) w << " " << (int) l << " " << (int) b << " " << (int) p << " " << shot_info_table[w][l][b][p].shot_obstructions.obstructing_object_balls.size() << endl;
          }
        }
      }
    }
  }
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}