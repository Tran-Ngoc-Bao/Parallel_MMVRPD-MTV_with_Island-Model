#pragma once
#include <vector>
#include <random>
#include <chrono>
#include <cstddef>
#include "config.hpp"
#include "population.hpp"

class Logger;

// -----------------------------------------------------------------------
// Island – one population of the Hybrid Genetic Search (Jamshidi et al.
// 2025, section 4) evolving until its stop condition:
//   4.1 Start          parameters (initial_pop, max_pop) from the config
//   4.2 Initialize     tabu-search initializer with seed + n
//   4.3 Population     genotype + phenotype, sorted by fitness, best first
//   4.4 Selection      two binary tournaments
//   4.5 Crossover      OX on genotypes + Split (genetics::decode)
//   4.6 Local search   Prins' moves, used as mutation
//   4.7 Insertion      sorted insert, no clones, size <= max_pop
//   4.8 End            repeat 4.4-4.7, report the first individual
// -----------------------------------------------------------------------
class Island {
public:
    Island();

    // Runs the HGS. Returns the best feasible solution found.
    Solution run(Logger& logger);

    const Solution&   best()        const { return best_; }
    const Population& population()  const { return pop_; }
    std::size_t generations()       const { return generations_; }
    std::size_t children()          const { return children_; }
    std::size_t last_improved()     const { return last_improved_; }
    std::size_t evaluations()       const { return evaluations_; }
    double elapsed() const;

    const std::vector<double>& time_checkpoints() const { return checkpoints_; }

private:
    std::mt19937_64 rng_;
    Population  pop_;
    Solution    best_;
    bool        has_best_      = false;
    std::size_t generations_   = 0;
    std::size_t children_      = 0;
    std::size_t last_improved_ = 0;
    std::size_t evaluations_   = 0;
    std::chrono::steady_clock::time_point start_;

    std::vector<double> checkpoints_;
    std::size_t next_checkpoint_ = 0;

    void initialize_population();
    void step();
    const Individual& select_parent();
    void consider_best(const Solution& s);
    void record_checkpoints();
    bool out_of_budget() const;
    bool should_stop() const;
};
