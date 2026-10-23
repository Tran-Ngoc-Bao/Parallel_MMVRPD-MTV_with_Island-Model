#pragma once
#include <vector>
#include <cstddef>
#include "solutions.hpp"

// Individual = the genotype (giant tour, used by crossover) and its
// phenotype (the decoded Solution, the same structure as a tabu-search
// elite).
struct Individual {
    std::vector<std::size_t> genotype;
    Solution phenotype;
};

// -----------------------------------------------------------------------
// Sections 4.3 and 4.7 – the population is a list sorted by ascending
// fitness (Solution::cost()), so the best individual is always first.
// -----------------------------------------------------------------------
class Population {
public:
    explicit Population(std::size_t max_size) : max_size_(max_size) {}

    // Inserts at the sorted position. Returns false (and inserts nothing)
    // when an identical solution exists. Past max_size the worst
    // individual is discarded.
    bool add(Individual ind);

    std::size_t size() const { return members_.size(); }
    bool empty() const { return members_.empty(); }
    const Individual& operator[](std::size_t i) const { return members_[i]; }
    const Individual& best() const { return members_.front(); }

    double average_cost() const;
    double feasible_ratio() const;

private:
    std::size_t max_size_;
    std::vector<Individual> members_;
};
