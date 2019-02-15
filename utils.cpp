#include "./constants.h"
#include <set>

using namespace std;

double from_diamonds(double diamonds) {
  return UNITS_PER_DIAMOND * diamonds;
}

double to_diamonds(double units) {
  return units / UNITS_PER_DIAMOND;
}

double diamonds(double diamonds) {
  return from_diamonds(diamonds);
}

void insert_into_set(set<short> &set1, set<short> &set2)
{
  for (short index : set2)
  {
    set1.insert(index);
  }
}
