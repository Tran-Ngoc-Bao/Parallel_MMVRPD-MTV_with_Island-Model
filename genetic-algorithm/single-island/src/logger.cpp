#include "logger.hpp"
#include "island.hpp"
#include "config.hpp"
#include <filesystem>
#include <random>
#include <stdexcept>
#include <iostream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

static std::string random_id(size_t len = 8) {
    static const char chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<> dist(0, sizeof(chars) - 2);
    std::string s;
    s.reserve(len);
    for (size_t i = 0; i < len; ++i) s += chars[dist(rng)];
    return s;
}

Logger::Logger()
{
    const Config& cfg = global_config();
    _outputs = cfg.outputs;
    fs::path out(cfg.outputs);
    if (!fs::is_directory(out)) fs::create_directories(out);

    _problem = fs::path(cfg.problem).stem().string();
    if (_problem.empty()) throw std::runtime_error("Cannot determine problem name");
    _id = cfg.run_id.empty() ? random_id(8) : cfg.run_id;

    if (!cfg.disable_logging && !cfg.compact_output) {
        fs::path csv = out / (_problem + "-" + _id + ".csv");
        std::ofstream f(csv);
        if (!f) throw std::runtime_error("Cannot create CSV: " + csv.string());
        std::cerr << "Logging generations to " << csv << "\n";
        f << "sep=,\n";
        f << "Generation,Offspring,Elapsed,Evaluations,Best cost,"
             "Population size,Population best cost,Population average cost,"
             "Population worst cost,Feasible ratio\n";
        _writer = std::move(f);
    }
}

void Logger::log_generation(const Island& island)
{
    if (!_writer) return;
    const Population& pop = island.population();
    *_writer
        << island.generations()       << ","
        << island.children()          << ","
        << island.elapsed()           << ","
        << island.evaluations()       << ","
        << island.best().cost()       << ","
        << pop.size()                 << ","
        << pop.best().phenotype.cost() << ","
        << pop.average_cost()         << ","
        << pop[pop.size() - 1].phenotype.cost() << ","
        << pop.feasible_ratio()       << "\n";
}

void Logger::finalize(const Island& island)
{
    const Config& cfg = global_config();
    if (cfg.disable_logging) return;

    auto sj = island.best().to_json();
    auto cj = config_to_json(cfg);

    nlohmann::json run;
    run["problem"]                    = _problem;
    run["algorithm"]                  = "ga-single-island";
    run["generations"]                = island.generations();
    run["iterations"]                 = island.children();
    run["last_improved"]              = island.last_improved();
    run["solution"]                   = sj;
    run["config"]                     = cj;
    run["elapsed"]                    = island.elapsed();
    run["total_evaluations"]          = island.evaluations();
    run["time_checkpoint_limit"]      = cfg.time_limit;
    run["best_solution_cost_by_time_checkpoint"] = island.time_checkpoints();

    fs::path out(_outputs);
    auto write = [&](const fs::path& p, const std::string& content) {
        std::ofstream f(p);
        if (!f) throw std::runtime_error("Cannot write " + p.string());
        f << content;
        std::cout << p << "\n";
    };

    write(out / (_problem + "-" + _id + ".json"), run.dump());
    if (!cfg.compact_output) {
        write(out / (_problem + "-" + _id + "-solution.json"), sj.dump());
        write(out / (_problem + "-" + _id + "-config.json"),   cj.dump());
    }
}
