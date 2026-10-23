#include "genetics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace genetics {

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr int TRUCK = 0, DRONE = 1;

// -----------------------------------------------------------------------
// Decoder – state of one decoding. Everything is computed lazily and
// cached, so the binary search on T reuses the work of previous T values.
// -----------------------------------------------------------------------
class Decoder {
public:
    // `relaxed` allows truck trips with violations (fallback).
    Decoder(const std::vector<std::size_t>& genotype, bool relaxed, std::size_t& evaluations)
        : g_(genotype), n_(genotype.size()), relaxed_(relaxed), evals_(evaluations)
    {
        const Config& cfg = global_config();
        fleet_[TRUCK] = cfg.trucks_count;
        fleet_[DRONE] = cfg.drones_count;
        for (int k = 0; k < 2; ++k) {
            trips_[k].resize(n_);
            trips_done_[k].assign(n_, false);
            blocks_[k].resize(n_ + 1);
        }
    }

    std::optional<Solution> solve();

private:
    // Labels of Prins' Split inside the block starting at `start`:
    // label[k - start] = min time of one vehicle serving genotype[start..k),
    // pred[k - start] = start of its last trip. Positions are processed
    // lazily up to `next`.
    struct Block {
        std::vector<double>      label;
        std::vector<std::size_t> pred;
        std::size_t              next = 0;
    };

    const std::vector<std::size_t>& g_;
    std::size_t  n_;
    bool         relaxed_;
    std::size_t& evals_;
    std::size_t  fleet_[2];

    // trips_[k][p][m] = time of the trip genotype[p..p+m] for vehicle type k.
    std::vector<std::vector<double>> trips_[2];
    std::vector<bool>                trips_done_[2];
    std::vector<Block>               blocks_[2];

    // F[t][d] = farthest position covered by t trucks and d drones (for the
    // last T given to feasible()); truck_[t][d] = whether the last block
    // was a truck block.
    std::vector<std::vector<std::size_t>> F_;
    std::vector<std::vector<bool>>        truck_;

    const std::vector<double>& trips(int k, std::size_t p);
    double vehicle_time(int k, std::size_t start, std::size_t end);
    std::size_t reach(int k, std::size_t start, double T);
    bool feasible(double T);
    Solution build();
};

template<typename R>
static double trip_time(const LocalRc<R>& r, bool& ok)
{
    ok = route_feasible(*r);
    return ok ? r->working_time() : fleet_cost({stats_of(*r)}, {});
}

const std::vector<double>& Decoder::trips(int k, std::size_t p)
{
    auto& out = trips_[k][p];
    if (trips_done_[k][p]) return out;
    trips_done_[k][p] = true;

    const Config& cfg = global_config();
    std::vector<std::size_t> buf{0, 0};
    for (std::size_t q = p; q < n_; ++q) {
        std::size_t c = g_[q];
        bool ok = true;
        double time;
        buf.insert(buf.end() - 1, c);
        if (k == TRUCK) {
            time = trip_time(TruckRoute::make(buf), ok);
        } else {
            if (!cfg.dronable[c] || (cfg.single_drone_route && q > p)) break;
            time = trip_time(DroneRoute::make(buf), ok);
        }
        ++evals_;
        // Longer trips only get worse: stop at the first violation, except
        // that a single-customer truck trip (and, when relaxed, any truck
        // trip) is kept at its penalized cost so that a decoding exists.
        if (!ok && !(k == TRUCK && (relaxed_ || q == p))) break;
        out.push_back(time);
    }
    return out;
}

double Decoder::vehicle_time(int k, std::size_t start, std::size_t end)
{
    Block& b = blocks_[k][start];
    if (b.label.empty()) {
        b.label = {0.0};
        b.pred  = {start};
    }
    auto label = [&](std::size_t i) { return i < b.label.size() ? b.label[i] : INF; };

    // A single-route vehicle (--single-truck-route) makes one trip only.
    const bool single_route = k == TRUCK && global_config().single_truck_route;
    while (start + b.next < end) {
        std::size_t p = start + b.next++;
        double base = label(p - start);
        if (base == INF || (single_route && p != start)) continue;
        const auto& t = trips(k, p);
        if (b.label.size() < p - start + t.size() + 1) {
            b.label.resize(p - start + t.size() + 1, INF);
            b.pred.resize(p - start + t.size() + 1, start);
        }
        for (std::size_t m = 0; m < t.size(); ++m) {
            std::size_t i = p - start + m + 1;   // trip genotype[p..p+m]
            if (base + t[m] < b.label[i]) {
                b.label[i] = base + t[m];
                b.pred[i]  = p;
            }
        }
    }
    return label(end - start);
}

// Farthest end such that one vehicle of type k serves genotype[start..end)
// within T. Relies on the time of a block never decreasing when it grows.
std::size_t Decoder::reach(int k, std::size_t start, double T)
{
    if (fleet_[k] == 0) return start;
    std::size_t end = start;
    while (end < n_ && vehicle_time(k, start, end + 1) <= T) ++end;
    return end;
}

bool Decoder::feasible(double T)
{
    const std::size_t nt = fleet_[TRUCK], nd = fleet_[DRONE];
    F_.assign(nt + 1, std::vector<std::size_t>(nd + 1, 0));
    truck_.assign(nt + 1, std::vector<bool>(nd + 1, false));
    for (std::size_t t = 0; t <= nt; ++t)
        for (std::size_t d = 0; d <= nd; ++d) {
            if (t == 0 && d == 0) continue;
            std::size_t by_truck = t > 0 ? reach(TRUCK, F_[t - 1][d], T) : 0;
            std::size_t by_drone = d > 0 ? reach(DRONE, F_[t][d - 1], T) : 0;
            truck_[t][d] = t > 0 && by_truck >= by_drone;
            F_[t][d]     = std::max(by_truck, by_drone);
        }
    return F_[nt][nd] == n_;
}

Solution Decoder::build()
{
    TruckRoutes tr(fleet_[TRUCK]);
    DroneRoutes dr(fleet_[DRONE]);

    std::size_t t = fleet_[TRUCK], d = fleet_[DRONE];
    while (t > 0 || d > 0) {
        bool is_truck = truck_[t][d];
        std::size_t end   = F_[t][d];
        std::size_t start = is_truck ? F_[t - 1][d] : F_[t][d - 1];
        int k = is_truck ? TRUCK : DRONE;

        // Trips of this block, from Prins' Split labels.
        vehicle_time(k, start, end);
        const Block& b = blocks_[k][start];
        for (std::size_t q = end; q > start; q = b.pred[q - start]) {
            std::size_t p = b.pred[q - start];
            std::vector<std::size_t> c{0};
            c.insert(c.end(), g_.begin() + static_cast<std::ptrdiff_t>(p),
                     g_.begin() + static_cast<std::ptrdiff_t>(q));
            c.push_back(0);
            if (is_truck) tr[t - 1].push_back(TruckRoute::make(std::move(c)));
            else          dr[d - 1].push_back(DroneRoute::make(std::move(c)));
        }
        if (is_truck) std::reverse(tr[t - 1].begin(), tr[t - 1].end());
        else          std::reverse(dr[d - 1].begin(), dr[d - 1].end());

        if (is_truck) --t; else --d;
    }
    ++evals_;
    return Solution::make(std::move(tr), std::move(dr));
}

std::optional<Solution> Decoder::solve()
{
    if (n_ == 0) {
        F_.assign(fleet_[TRUCK] + 1, std::vector<std::size_t>(fleet_[DRONE] + 1, 0));
        truck_.assign(fleet_[TRUCK] + 1, std::vector<bool>(fleet_[DRONE] + 1, false));
        return build();
    }

    // Lower bound: every customer needs at least its cheapest single trip.
    double lo = 0.0;
    for (std::size_t p = 0; p < n_; ++p) {
        double best = INF;
        for (int k = 0; k < 2; ++k)
            if (fleet_[k] > 0 && !trips(k, p).empty()) best = std::min(best, trips(k, p)[0]);
        if (best == INF) return std::nullopt;
        lo = std::max(lo, best);
    }

    if (feasible(lo)) return build();

    double hi = std::max(lo, 1.0) * 2.0;
    for (int i = 0; i < 64 && !feasible(hi); ++i) hi *= 2.0;
    if (!feasible(hi)) return std::nullopt;

    // Invariant: lo infeasible, hi feasible.
    for (int i = 0; i < 100 && hi - lo > 1e-9 * hi; ++i) {
        double mid = 0.5 * (lo + hi);
        if (feasible(mid)) hi = mid;
        else               lo = mid;
    }
    feasible(hi);
    return build();
}

double polar_angle(const std::vector<std::size_t>& customers)
{
    const Config& cfg = global_config();
    double sx = 0, sy = 0;
    for (std::size_t c : customers) { sx += cfg.x[c]; sy += cfg.y[c]; }
    double m = static_cast<double>(customers.size());
    return std::atan2(sy / m - cfg.y[0], sx / m - cfg.x[0]);
}

} // namespace

Solution decode(const std::vector<std::size_t>& genotype, std::size_t& evaluations)
{
    if (auto s = Decoder(genotype, false, evaluations).solve()) return *s;
    if (auto s = Decoder(genotype, true,  evaluations).solve()) return *s;
    throw std::runtime_error("Split: the genotype cannot be decoded into trips");
}

std::vector<std::size_t> encode(const Solution& phenotype)
{
    struct Vehicle {
        double angle = 0;
        std::vector<std::pair<double, std::vector<std::size_t>>> trips;   // (angle, customers)
    };
    std::vector<Vehicle> vehicles;

    auto add_vehicle = [&](const auto& routes) {
        if (routes.empty()) return;
        Vehicle v;
        std::vector<std::size_t> all;
        for (const auto& r : routes) {
            const auto& c = r->data().customers;
            std::vector<std::size_t> trip(c.begin() + 1, c.end() - 1);
            all.insert(all.end(), trip.begin(), trip.end());
            v.trips.emplace_back(polar_angle(trip), std::move(trip));
        }
        v.angle = polar_angle(all);
        std::stable_sort(v.trips.begin(), v.trips.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });
        vehicles.push_back(std::move(v));
    };
    for (const auto& routes : phenotype.truck_routes) add_vehicle(routes);
    for (const auto& routes : phenotype.drone_routes) add_vehicle(routes);

    std::stable_sort(vehicles.begin(), vehicles.end(),
                     [](const Vehicle& a, const Vehicle& b) { return a.angle < b.angle; });

    std::vector<std::size_t> genotype;
    genotype.reserve(global_config().customers_count);
    for (const auto& v : vehicles)
        for (const auto& [_, trip] : v.trips)
            genotype.insert(genotype.end(), trip.begin(), trip.end());
    return genotype;
}

} // namespace genetics
