#ifndef UTILS_H
#define UTILS_H

#include "./constants.h"

/** Converts diamonds to units. */
double from_diamonds(double diamonds);

/** Converts units to diamonds. */
double to_diamonds(double units);

/** Shorthand for {@code from_diamonds} */
double diamonds(double diamonds);

/**
 * Modifies set 1 by adding all the elements from set 2.
 */
void insert_into_set(set<short> &set1, set<short> &set2);

#endif