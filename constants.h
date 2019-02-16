#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <limits>
#include <string>

using namespace std;

/**
 * The name of the file to output game data.
 */
const string GAME_DATA_FILE = "game_data.js";
/**
 * The number of game units within one diamond length on the pool table.
 */
const short UNITS_PER_DIAMOND = 4;
/**
 * The number of shot strengths to consider.
 */
const short NUM_STRENGTHS = 12;
/**
 * The number of diamonds in a meter.
 */
const double DIAMONDS_PER_METER = 3.14960629921;
/**
 * The number of units per meter.
 */
const double UNITS_PER_METER = UNITS_PER_DIAMOND * DIAMONDS_PER_METER;
/**
 * A number epsilon used to deal with doubles.
 */
const double EPSILON = 0.00000001;
/**
 * The table edge.
 */
enum Edge { LEFT, RIGHT, TOP, BOTTOM };
/**
 * The shot spins to consider
 */
enum Spin { HEAVY_DRAW, LIGHT_DRAW, STUN, LIGHT_FOLLOW, HEAVY_FOLLOW, MAX_NUM_SPINS };
/**
 * The shot speeds to consider. This is used for effective pocket size calculations.
 */
enum Speed { SLOW, FAST };
/**
 * The diameter of the balls in units.
 */
const double BALL_DIAMETER = 0.18 * UNITS_PER_DIAMOND;
/**
 * The radius of the balls in units.
 */
const double BALL_RADIUS = BALL_DIAMETER / 2;
/**
 * The diameter of the balls in diamonds.
 */
const double BALL_DIAMETER_IN_DIAMONDS = BALL_DIAMETER / UNITS_PER_DIAMOND;
/**
 * The radius of the balls in diamonds.
 */
const double BALL_RADIUS_IN_DIAMONDS = BALL_RADIUS / UNITS_PER_DIAMOND;
/**
 * The radius of the balls in meters.
 */
const double BALL_RADIUS_IN_METERS = BALL_RADIUS / UNITS_PER_METER;
/**
 * The width of the pool table in units.
 */
const short WIDTH = UNITS_PER_DIAMOND * 4;
/**
 * The length of the pool table in units.
 */
const short LENGTH = WIDTH * 2;
/**
 * The minimum speed shot to consider. (In meters/second)
 */
const double MIN_CUE_BALL_SPEED = 0.3;
/**
 * The maximum speed shot to consider. (In meters/second)
 */
const double MAX_CUE_BALL_SPEED = 4.5;
/**
 * The coefficient of friction of the cue ball table. Used in physics calculations.
 */
const double U = 0.2;
/**
 * Gravitational force constant. In meters/second.
 */
const double G = 9.807;

#endif