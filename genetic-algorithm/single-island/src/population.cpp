#include "population.hpp"
#include <algorithm>

static constexpr double SAME_COST_EPS = 1e-9;

bool Population::add(Individual ind)
{
    const double cost = ind.phenotype.cost();
    auto by_cost = [](const Individual& m, double c) { return m.phenotype.cost() < c; };

    // (1) Identical solutions: same cost and the same successor of every customer.
    auto it = std::lower_bound(members_.begin(), members_.end(), cost - SAME_COST_EPS, by_cost);
    for (auto k = it; k != members_.end() && k->phenotype.cost() <= cost + SAME_COST_EPS; ++k)
        if (k->phenotype.hamming_distance(ind.phenotype) == 0) return false;

    // (2) Keep the size limit by discarding the worst fitness.
    if (members_.size() >= max_size_) {
        if (cost >= members_.back().phenotype.cost()) return false;
        members_.pop_back();
    }
    it = std::lower_bound(members_.begin(), members_.end(), cost, by_cost);
    members_.insert(it, std::move(ind));
    return true;
}

double Population::average_cost() const
{
    if (members_.empty()) return 0.0;
    double s = 0.0;
    for (const auto& m : members_) s += m.phenotype.cost();
    return s / static_cast<double>(members_.size());
}

double Population::feasible_ratio() const
{
    if (members_.empty()) return 0.0;
    std::size_t f = 0;
    for (const auto& m : members_) if (m.phenotype.feasible) ++f;
    return static_cast<double>(f) / static_cast<double>(members_.size());
}
