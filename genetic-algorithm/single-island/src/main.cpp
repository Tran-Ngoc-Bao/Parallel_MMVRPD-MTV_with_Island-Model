#include <CLI/CLI.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>

#include "cli.hpp"
#include "config.hpp"
#include "solutions.hpp"
#include "logger.hpp"
#include "island.hpp"

int main(int argc, char** argv)
{
    CLI::App app{"Single-island Hybrid Genetic Search for VRP with Drones"};
    app.require_subcommand(1);

    cli::RunArgs args;

    auto* run_cmd = app.add_subcommand("run", "Run the single-island HGS");
    run_cmd->add_option("problem", args.problem, "Problem file path")->required();
    run_cmd->add_option("--truck-cfg",  args.truck_cfg);
    run_cmd->add_option("--drone-cfg",  args.drone_cfg);

    std::map<std::string, cli::EnergyModel> em_map{
        {"linear", cli::EnergyModel::Linear},
        {"non-linear", cli::EnergyModel::NonLinear},
        {"endurance", cli::EnergyModel::Endurance},
        {"unlimited", cli::EnergyModel::Unlimited}
    };
    run_cmd->add_option("-c,--config", args.config)->transform(
        CLI::CheckedTransformer(em_map, CLI::ignore_case));

    std::map<std::string, cli::ConfigType> ct_map{
        {"low", cli::ConfigType::Low}, {"high", cli::ConfigType::High}
    };
    run_cmd->add_option("--speed-type", args.speed_type)->transform(
        CLI::CheckedTransformer(ct_map, CLI::ignore_case));
    run_cmd->add_option("--range-type", args.range_type)->transform(
        CLI::CheckedTransformer(ct_map, CLI::ignore_case));

    std::map<std::string, cli::DistanceType> dt_map{
        {"manhattan", cli::DistanceType::Manhattan},
        {"euclidean", cli::DistanceType::Euclidean}
    };
    run_cmd->add_option("--truck-distance", args.truck_distance)->transform(
        CLI::CheckedTransformer(dt_map, CLI::ignore_case));
    run_cmd->add_option("--drone-distance", args.drone_distance)->transform(
        CLI::CheckedTransformer(dt_map, CLI::ignore_case));

    std::optional<size_t> trucks_count_cli, drones_count_cli;
    run_cmd->add_option("--trucks-count", trucks_count_cli);
    run_cmd->add_option("--drones-count", drones_count_cli);
    run_cmd->add_option("--waiting-time-limit", args.waiting_time_limit);
    run_cmd->add_option("--time-limit",         args.time_limit,
        "If non-zero, stop after this many seconds");
    run_cmd->add_option("--max-evaluations",    args.max_evaluations,
        "If non-zero, stop after this many total evaluations. Stop-condition "
        "priority: max-evaluations, then time-limit, then --max-stagnation");
    run_cmd->add_option("--penalty-exponent",   args.penalty_exponent);
    run_cmd->add_flag  ("--single-truck-route", args.single_truck_route);
    run_cmd->add_flag  ("--single-drone-route", args.single_drone_route);
    run_cmd->add_flag  ("-v,--verbose",          args.verbose);
    run_cmd->add_option("--outputs",             args.outputs);
    run_cmd->add_flag  ("--disable-logging",     args.disable_logging);
    run_cmd->add_flag  ("--dry-run",             args.dry_run);
    run_cmd->add_option("--extra",               args.extra);
    run_cmd->add_flag  ("--compact-output",      args.compact_output);
    run_cmd->add_option("--run-id",              args.run_id);
    std::optional<uint64_t> seed_cli;
    run_cmd->add_option("--seed", seed_cli);

    // ---- HGS parameters ----
    run_cmd->add_option("--initial-pop",  args.initial_pop,
        "Individuals built by the tabu-search initializer, with seed, seed+1, ...");
    run_cmd->add_option("--max-pop",      args.max_pop,
        "Population size limit (0 = 2 * initial-pop)");
    run_cmd->add_option("--ls-neighbors", args.ls_neighbors,
        "Local search: nearest neighbours tried per customer (0 = all)");
    run_cmd->add_option("--max-stagnation",    args.max_stagnation,
        "Stop after this many offspring without improvement (only when "
        "neither --time-limit nor --max-evaluations is set)");
    run_cmd->add_option("--max-iterations",    args.max_iterations,
        "Stop after this many offspring (0 = unlimited)");

    CLI11_PARSE(app, argc, argv);

    if (trucks_count_cli) args.trucks_count = trucks_count_cli;
    if (drones_count_cli) args.drones_count = drones_count_cli;
    if (seed_cli)         args.seed         = seed_cli;
    if (args.initial_pop < 2) { std::cerr << "--initial-pop must be >= 2\n"; return 1; }
    if (args.max_pop != 0 && args.max_pop < args.initial_pop) {
        std::cerr << "--max-pop must be >= --initial-pop\n"; return 1;
    }

    set_global_config(build_config(args));
    Logger logger;

    auto t0 = std::chrono::steady_clock::now();
    Island island;
    Solution solution = island.run(logger);
    auto t1 = std::chrono::steady_clock::now();

    logger.finalize(island);

    double total_sec = std::chrono::duration<double>(t1 - t0).count();
    std::cerr << std::fixed << std::setprecision(6)
              << "Timing (mode=ga-single-island, unit=s): total=" << total_sec << "\n"
              << "Generations = " << island.generations()
              << ", offspring = " << island.children()
              << ", evaluations = " << island.evaluations() << "\n";

    std::cerr << "Result = " << solution.working_time << "\n";
    solution.verify();
    return 0;
}
