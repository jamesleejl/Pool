#ifndef PRECOMPUTED_H
#define PRECOMPUTED_H

#include <map>

using namespace std;

/**
 * A map from shot angle (in degrees) multiplied by ten to the effective pocket size
 * in diamonds as defined by
 * https://billiards.colostate.edu/technical_proofs/TP_3-4.pdf.
 * This is for a slow shot into the side pocket.
 */
extern map<short, double> deciangle_to_effective_pocket_size_slow_side;
/**
 * A map from shot angle (in degrees) multiplied by ten to the effective pocket size
 * in diamonds as defined by
 * https://billiards.colostate.edu/technical_proofs/TP_3-4.pdf.
 * This is for a fast shot into the side pocket.
 */
extern map<short, double> deciangle_to_effective_pocket_size_fast_side;
/**
 * A map from shot angle (in degrees) multiplied by ten to the effective pocket size
 * in diamonds as defined by
 * https://billiards.colostate.edu/technical_proofs/TP_3-4.pdf.
 * This is for a fast shot into the corner pocket.
 */
extern map<short, double> deciangle_to_effective_pocket_size_fast_corner;
/**
 * A map from shot angle (in degrees) multiplied by ten to the effective pocket size
 * in diamonds as defined by
 * https://billiards.colostate.edu/technical_proofs/TP_3-4.pdf.
 * This is for a slow shot into the corner pocket.
 */
extern map<short, double> deciangle_to_effective_pocket_size_slow_corner;
/**
 * Gets the effective pocket size given the deciangle. If the angle is not found in the map, returns an effective size of 0.
 */
double deciangle_to_effective_pocket_size(short deciangle, const map<short, double>& deciangle_to_effective_pocket_size);

#endif