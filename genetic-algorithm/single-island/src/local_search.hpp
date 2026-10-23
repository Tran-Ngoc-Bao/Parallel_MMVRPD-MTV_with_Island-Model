#pragma once
#include <random>
#include <cstddef>
#include "solutions.hpp"

// -----------------------------------------------------------------------
// Section 4.6 – local search used as the mutation operator (Prins 2004).
//
// For a customer u with successor x, and a neighbour v with successor y
// (x, y may be the depot), the moves are tried in this order:
//   M1 Relocate  u after v            M4 Swap u and v
//   M2 Relocate  (u, x) after v       M5 Swap (u, x) and v
//   M3 Relocate  (x, u) after v       M6 Swap (u, x) and (v, y)
//   M7 2-Opt   (same trip):      (u, x), (v, y) -> (u, v), (x, y)
//   M8 2-Opt*  (different trips): (u, x), (v, y) -> (u, v), (x, y)
//   M9 2-Opt*  (different trips): (u, x), (v, y) -> (u, y), (x, v)
// plus relocating u into a new trip of the least-loaded truck / drone,
// since here a vehicle can make several trips.
//
// Trips may be on any truck or drone. A move is applied as soon as it
// improves (first improvement), and passes are repeated until none does.
// Only moves that keep every changed trip free of violations are
// accepted, so feasible solutions stay feasible. "Improves" means a
// lower makespan, or the same makespan with a lower total working time
// (otherwise moves on non-critical vehicles would never be accepted).
// -----------------------------------------------------------------------
namespace local_search {

Solution improve(const Solution& s, std::mt19937_64& rng, std::size_t& evaluations);

} // namespace local_search
