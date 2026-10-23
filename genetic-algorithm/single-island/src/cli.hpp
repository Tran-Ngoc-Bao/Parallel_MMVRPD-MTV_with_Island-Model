#pragma once
#include <string>
#include <optional>
#include <cstdint>

namespace cli {

enum class EnergyModel { Linear = 0, NonLinear = 1, Endurance = 2, Unlimited = 3 };
enum class ConfigType  { Low, High };
enum class DistanceType{ Manhattan, Euclidean };

inline const char* to_str(EnergyModel m){
    switch(m){
        case EnergyModel::Linear:    return "linear";
        case EnergyModel::NonLinear: return "non-linear";
        case EnergyModel::Endurance: return "endurance";
        case EnergyModel::Unlimited: return "unlimited";
    }
    return "";
}
inline const char* to_str(ConfigType c){
    return c == ConfigType::Low ? "low" : "high";
}
inline const char* to_str(DistanceType d){
    return d == DistanceType::Manhattan ? "manhattan" : "euclidean";
}

// ---------------------------------------------------------------------
// Run options: problem / stop conditions (as in the tabu search) + HGS
// ---------------------------------------------------------------------
struct RunArgs {
    std::string   problem;
    std::string   truck_cfg                  = "problems/config_parameter/truck_config.json";
    std::string   drone_cfg                  = "problems/config_parameter/drone_endurance_config.json";
    EnergyModel   config                     = EnergyModel::Endurance;
    ConfigType    speed_type                 = ConfigType::High;
    ConfigType    range_type                 = ConfigType::High;
    DistanceType  truck_distance             = DistanceType::Euclidean;
    DistanceType  drone_distance             = DistanceType::Euclidean;
    std::optional<std::size_t> trucks_count  = std::nullopt;
    std::optional<std::size_t> drones_count  = std::nullopt;
    double        waiting_time_limit         = 3600.0;
    double        time_limit                 = 0.0;
    std::size_t   max_evaluations            = 0;
    double        penalty_exponent           = 0.5;
    bool          single_truck_route         = false;
    bool          single_drone_route         = false;
    bool          verbose                    = false;
    std::string   outputs                    = "outputs/";
    bool          disable_logging            = false;
    bool          dry_run                    = false;
    std::string   extra                      = "";
    std::optional<uint64_t> seed             = std::nullopt;
    bool          compact_output             = false;
    std::string   run_id                     = "";

    // Hybrid Genetic Search (section 4.1 of Jamshidi et al. 2025)
    std::size_t   initial_pop                = 300;   // individuals built by the tabu-search initializer (seed, seed+1, ...)
    std::size_t   max_pop                    = 0;     // population cap, 0 = 2 * initial_pop
    std::size_t   ls_neighbors               = 20;    // local search: nearest neighbours tried per customer, 0 = all
    std::size_t   max_stagnation             = 5000;  // offspring without improvement (only without time/eval limit)
    std::size_t   max_iterations             = 0;     // offspring cap, 0 = unlimited
};

} // namespace cli
