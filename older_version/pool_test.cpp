TEST_F(PoolTest, PopulateShotInfoTableObstructions)
{
  eight_ball = Vector2d(UNITS_PER_DIAMOND, UNITS_PER_DIAMOND * 6);
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 2, UNITS_PER_DIAMOND * 4));
  object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 3, UNITS_PER_DIAMOND * 2));
  opponent_object_balls.push_back(Vector2d(UNITS_PER_DIAMOND * 3, UNITS_PER_DIAMOND * 4));
  initialize_pockets();
  initialize_table_edges();
  populate_ball_to_pocket_obstructions_table();
  populate_ghost_ball_position_table();
  populate_shot_info_table_obstructions();
  for (short w = 0; w <= WIDTH; ++w)
  {
    for (short l = 0; l <= LENGTH; ++l)
    {
      for (short b = 0; b < object_balls.size() + 1; ++b)
      {
        for (short p = 0; p < 6; ++p)
        {
          if (b == 0 && (p == 3 || p == 4))
          {
            EXPECT_TRUE(shot_info_table[w][l][b][p].shot_obstructions.has_permanent_obstruction);
          }
        }
      }
    }
  }
}
