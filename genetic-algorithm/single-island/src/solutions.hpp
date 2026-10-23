#pragma once
#include <vector>
#include <memory>
#include <string>
#include <atomic>
#include <cstdint>
#include <nlohmann/json.hpp>
#include "routes.hpp"

// -----------------------------------------------------------------------
// Penalty coefficients – global atomics
// -----------------------------------------------------------------------
namespace penalty {
    extern std::atomic<double> coeff[4];
    inline double get(int i) { return coeff[i].load(std::memory_order_relaxed); }
    inline void   set(int i, double v) { coeff[i].store(v, std::memory_order_relaxed); }
}

// -----------------------------------------------------------------------
// Solution
// -----------------------------------------------------------------------
struct Solution {
    std::vector<std::vector<LocalRc<TruckRoute>>> truck_routes;
    std::vector<std::vector<LocalRc<DroneRoute>>> drone_routes;

    std::vector<double> truck_working_time;
    std::vector<double> drone_working_time;

    // Per-vehicle breakdown (kept in sync by make()); same fields as the
    // tabu-search Solution so individuals and elites stay interchangeable.
    std::vector<double> truck_capacity_violation;
    std::vector<double> truck_waiting_violation;
    std::vector<double> drone_capacity_violation;
    std::vector<double> drone_waiting_violation;
    std::vector<double> drone_energy_violation;
    std::vector<double> drone_fixed_violation;

    double working_time           = 0;
    double energy_violation       = 0;
    double capacity_violation     = 0;
    double waiting_time_violation = 0;
    double fixed_time_violation   = 0;
    bool   feasible               = false;

    // Factory – computes all derived fields.
    static Solution make(
        std::vector<std::vector<LocalRc<TruckRoute>>> truck_routes,
        std::vector<std::vector<LocalRc<DroneRoute>>> drone_routes);

    double cost() const;

    size_t hamming_distance(const Solution& other) const;

    void verify() const;

    // Build initial solution from problem data (tabu-search initializer).
    // `seed` drives the shuffling of each cluster.
    static Solution initialize(uint64_t seed);

    // JSON serialization
    nlohmann::json to_json() const;
};
