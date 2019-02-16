#ifndef UTILS_H
#define UTILS_H

#include "./constants.h"
#include "Eigen/Dense"

/** Converts diamonds to units. */
double from_diamonds(double diamonds);

/** Converts units to diamonds. */
double to_diamonds(double units);

/** Shorthand for {@code from_diamonds} */
double diamonds(double diamonds);
/** Converts the vector from units to diamonds */
Vector2d to_diamonds(const Vector2d& units_vec);
/**
 * Modifies set 1 by adding all the elements from set 2.
 */
void insert_into_set(set<short> &set1, set<short> &set2);

#endif