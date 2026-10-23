#pragma once
#include <vector>
#include <random>
#include <cstddef>

// -----------------------------------------------------------------------
// Section 4.5 of Jamshidi et al. 2025 – crossover of two parent
// genotypes; `genetics::decode` turns the child into its phenotype.
// -----------------------------------------------------------------------
namespace crossover {

// Ordered crossover (OX): copy a random slice of p1 to the same place,
// then fill the rest with p2's order, circularly from the second cut point.
std::vector<std::size_t> ordered(const std::vector<std::size_t>& p1,
                                 const std::vector<std::size_t>& p2,
                                 std::mt19937_64& rng);

} // namespace crossover
