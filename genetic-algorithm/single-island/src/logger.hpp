#pragma once
#include <string>
#include <fstream>
#include <optional>

class Island;

// -----------------------------------------------------------------------
// Logger – per-generation CSV + final JSON. The JSON keeps the same
// keys as the tabu-search output (solution, config, elapsed,
// total_evaluations, best_solution_cost_by_time_checkpoint, ...) so
// results of both algorithms can be compared with the same scripts.
// -----------------------------------------------------------------------
class Logger {
public:
    Logger();

    void log_generation(const Island& island);
    void finalize(const Island& island);

private:
    std::string _outputs;
    std::string _problem;
    std::string _id;
    std::optional<std::ofstream> _writer;
};
