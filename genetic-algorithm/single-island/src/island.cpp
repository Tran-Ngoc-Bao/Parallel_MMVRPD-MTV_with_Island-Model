#include "island.hpp"
#include "crossover.hpp"
#include "genetics.hpp"
#include "local_search.hpp"
#include "logger.hpp"
#include <cmath>
#include <iostream>
#include <limits>

static constexpr double TOLERANCE = 0.001;

// 4.1 Start: the parameters come from global_config().
Island::Island()
    : pop_(global_config().max_pop)
{
    const Config& cfg = global_config();
    if (cfg.seed) rng_.seed(*cfg.seed);
    else          rng_.seed(std::random_device{}());

    for (int i = 0; i < 4; ++i) penalty::set(i, 1.0);
}

double Island::elapsed() const
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
}

bool Island::out_of_budget() const
{
    const Config& cfg = global_config();
    if (cfg.max_evaluations > 0 && evaluations_ >= cfg.max_evaluations) return true;
    if (cfg.time_limit > 0.0 && elapsed() >= cfg.time_limit) return true;
    return false;
}

// Stop-condition priority mirrors the tabu search: max-evaluations, then
// time-limit, then offspring without improvement.
bool Island::should_stop() const
{
    const Config& cfg = global_config();
    if (cfg.max_iterations > 0 && children_ >= cfg.max_iterations) return true;
    if (cfg.max_evaluations > 0 || cfg.time_limit > 0.0) return out_of_budget();
    return children_ - last_improved_ >= cfg.max_stagnation;
}

void Island::record_checkpoints()
{
    const Config& cfg = global_config();
    if (cfg.time_limit <= 0.0) return;
    double e = elapsed();
    double cost = has_best_ ? best_.cost() : std::numeric_limits<double>::infinity();
    while (next_checkpoint_ <= 8 &&
           e >= cfg.time_limit * static_cast<double>(next_checkpoint_) / 8.0) {
        checkpoints_[next_checkpoint_] = cost;
        ++next_checkpoint_;
    }
}

void Island::consider_best(const Solution& s)
{
    if (!s.feasible) return;
    if (!has_best_ || s.cost() + TOLERANCE < best_.cost()) {
        best_ = s;
        has_best_ = true;
        last_improved_ = children_;
    }
}

// 4.2 Initialize individuals: the tabu-search initializer, run with seed
// base, base + 1, base + 2, ... (base = --seed, or a random value). Clones
// are rejected by the population, so give up after 10 * initial_pop seeds.
void Island::initialize_population()
{
    const Config& cfg = global_config();
    const uint64_t base = cfg.seed ? *cfg.seed : std::random_device{}();
    for (uint64_t n = 0; pop_.size() < cfg.initial_pop && n < 10 * cfg.initial_pop &&
                         (pop_.size() < 2 || !out_of_budget()); ++n) {
        Individual ind;
        ind.phenotype = Solution::initialize(base + n);
        ind.genotype  = genetics::encode(ind.phenotype);
        ++evaluations_;
        consider_best(ind.phenotype);
        pop_.add(std::move(ind));
    }
}

// 4.4 Selection: binary tournament between two distinct individuals. The
// population is sorted, so the lower index has the better fitness.
const Individual& Island::select_parent()
{
    std::uniform_int_distribution<std::size_t> pick(0, pop_.size() - 1);
    std::size_t a = pick(rng_), b = pick(rng_);
    while (b == a) b = pick(rng_);
    return pop_[std::min(a, b)];
}

void Island::step()
{
    // 4.4 Selection
    const Individual& p1 = select_parent();
    const Individual& p2 = select_parent();

    // 4.5 Crossover (OX) + Split
    Individual child;
    std::vector<std::size_t> genotype = crossover::ordered(p1.genotype, p2.genotype, rng_);
    Solution decoded = genetics::decode(genotype, evaluations_);

    // 4.6 Local search as mutation; the genotype is re-encoded from the
    // improved phenotype.
    child.phenotype = local_search::improve(decoded, rng_, evaluations_);
    child.genotype  = genetics::encode(child.phenotype);

    ++children_;
    consider_best(child.phenotype);

    // 4.7 Insertion
    pop_.add(std::move(child));
}

Solution Island::run(Logger& logger)
{
    const Config& cfg = global_config();
    start_ = std::chrono::steady_clock::now();

    if (cfg.time_limit > 0.0) checkpoints_.assign(9, 0.0);

    initialize_population();
    if (pop_.size() < 2)
        throw std::runtime_error("The initializer produced fewer than 2 distinct individuals");
    record_checkpoints();
    logger.log_generation(*this);

    if (!cfg.dry_run) {
        // 4.8 End: repeat 4.4-4.7 until the stop condition. A generation is
        // counted every initial_pop offspring (for logging only).
        while (!should_stop()) {
            record_checkpoints();
            step();

            if (children_ % cfg.initial_pop == 0) {
                ++generations_;
                logger.log_generation(*this);
            }

            if (cfg.verbose)
                std::cerr << "Generation #" << generations_
                          << " offspring " << children_
                          << " best " << (has_best_ ? best_.cost() : pop_.best().phenotype.cost())
                          << " pop " << pop_.size()
                          << " avg " << pop_.average_cost()
                          << "     \r";
        }
        if (cfg.verbose) std::cerr << "\n";
    }

    // The paper reports the first individual of the sorted population; we
    // report the best feasible solution seen, which is the same whenever
    // the first individual is feasible.
    if (!has_best_) best_ = pop_.best().phenotype;

    if (cfg.time_limit > 0.0) {
        while (next_checkpoint_ <= 8) checkpoints_[next_checkpoint_++] = best_.cost();
        checkpoints_[8] = best_.cost();
    }
    return best_;
}
