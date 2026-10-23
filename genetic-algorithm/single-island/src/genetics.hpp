#pragma once
#include <vector>
#include <cstddef>
#include "solutions.hpp"

// -----------------------------------------------------------------------
// Genotype <-> phenotype (section 4.3 of Jamshidi et al. 2025).
//
// The genotype of an individual is a giant tour: a permutation of all
// customers without trip or vehicle delimiters. `decode` turns it into
// the phenotype, a Solution (the same structure as a tabu-search elite);
// `encode` writes a phenotype back into a genotype, e.g. after local
// search.
//
// Each vehicle serves one contiguous block of the genotype, so that
// decode(encode(s)) never has a larger makespan than s.
// -----------------------------------------------------------------------
namespace genetics {

// Split for min-makespan with trucks + drones and multiple trips. Cuts the
// genotype into at most trucks_count + drones_count contiguous blocks, one
// per vehicle, each block served by its vehicle in consecutive trips:
//  - the time of one vehicle for a block is the sum of its trip times,
//    minimized exactly by Prins' Split inside the block (every trip free
//    of violations; drones only serve dronable customers);
//  - the smallest makespan T for which the blocks can cover the whole
//    genotype is found by binary search; for a given T, F[t][d] = the
//    farthest position covered by t trucks and d drones.
// If no violation-free decoding exists (e.g. --single-truck-route with too
// few vehicles), truck trips with violations are allowed, at their
// penalized cost.
Solution decode(const std::vector<std::size_t>& genotype, std::size_t& evaluations);

// Concatenates the trips vehicle by vehicle: vehicles ordered by the polar
// angle of the centroid of all their customers around the depot, and the
// trips of a vehicle by the polar angle of their own centroid.
std::vector<std::size_t> encode(const Solution& phenotype);

} // namespace genetics
