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
    obstructions empty_obstructions;
    for (unsigned char b = 0; b < 8; ++b) {
      for (unsigned char p = 0; p < 6; ++p) {
        ball_to_pocket_obstructions_table[b][p] = empty_obstructions;
        ghost_ball_position_table[b][p] = Vector2d(0,0);
        for (unsigned char w = 0; w < WIDTH; ++w) {
          for (unsigned char l = 0; l < LENGTH; ++l) {
            shot_info_table[w][l][b][p] = shot_info();
            for (unsigned char st = 0; st < NUM_STRENGTHS; ++st) {
              for (unsigned char sp = 0; sp < NUM_SPINS; ++ sp) {
                shot_path_table[w][l][b][p][st][sp] = shot_path();
              }
            }
          }
        }
      }
    }
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
      EXPECT_TRUE(ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p].has_permanent_obstruction);
    } else {
      EXPECT_FALSE(ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p].has_permanent_obstruction);
    }
    if (p == 4) {
      EXPECT_EQ(1, ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p].obstructing_object_balls.size());
      EXPECT_TRUE(ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p].obstructing_object_balls.find(1) != ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p].obstructing_object_balls.end());
    } else {
      EXPECT_EQ(0, ball_to_pocket_obstructions_table[EIGHT_BALL_INDEX][p].obstructing_object_balls.size());
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

/*
// TODO: Uncomment this. This test is incomplete.
TEST_F(PoolTest, PopulateShotInfoTableObstructions) {
  initialize_pockets();
  eight_ball = Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 6);
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 2, DIAMOND_LENGTH * 4));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH * 0.5, DIAMOND_LENGTH * 7));
  object_balls.push_back(Vector2d(DIAMOND_LENGTH, DIAMOND_LENGTH * 4));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 4));
  opponent_object_balls.push_back(Vector2d(DIAMOND_LENGTH * 3, DIAMOND_LENGTH * 2));
  populate_ball_to_pocket_obstructions_table();
  populate_shot_info_table_obstructions();
  for (unsigned char p = 0; p < 6; ++p) {
    for (unsigned char b = 0; b < 3; ++b) {
      for (unsigned char w = 0; w < WIDTH; ++w) {
        for (unsigned char l = 0; l < LENGTH; ++l) {
          // cout << (int) b << " " << (int) p << " " << (int) w << " " << (int) l << " " << endl;
          EXPECT_FALSE(shot_info_table[b][p][w][l].shot_obstructions.has_permanent_obstruction);
        }
      }
    }
  }
}
*/
}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}