#include "solutions.hpp"
#include "clusterize.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <queue>
#include <random>

// -----------------------------------------------------------------------
// Penalty coefficients
// -----------------------------------------------------------------------
namespace penalty {
    std::atomic<double> coeff[4] = {
        {1.0}, {1.0}, {1.0}, {1.0}
    };
}

// -----------------------------------------------------------------------
// Solution::make
// -----------------------------------------------------------------------
Solution Solution::make(
    std::vector<std::vector<LocalRc<TruckRoute>>> truck_routes,
    std::vector<std::vector<LocalRc<DroneRoute>>> drone_routes)
{
    const Config& cfg = global_config();
    Solution sol;
    sol.truck_routes = std::move(truck_routes);
    sol.drone_routes = std::move(drone_routes);

    double working_time           = 0.0;
    double energy_violation       = 0.0;
    double capacity_violation     = 0.0;
    double waiting_time_violation = 0.0;
    double fixed_time_violation   = 0.0;

    sol.truck_working_time.resize(sol.truck_routes.size(), 0.0);
    sol.drone_working_time.resize(sol.drone_routes.size(), 0.0);
    sol.truck_capacity_violation.resize(sol.truck_routes.size(), 0.0);
    sol.truck_waiting_violation.resize(sol.truck_routes.size(), 0.0);
    sol.drone_capacity_violation.resize(sol.drone_routes.size(), 0.0);
    sol.drone_waiting_violation.resize(sol.drone_routes.size(), 0.0);
    sol.drone_energy_violation.resize(sol.drone_routes.size(), 0.0);
    sol.drone_fixed_violation.resize(sol.drone_routes.size(), 0.0);

    for (size_t t = 0; t < sol.truck_routes.size(); ++t) {
        double sum = 0.0;
        // Per-vehicle cache, accumulated in parallel with (but not used
        // in place of) the running totals below, so the totals' exact
        // floating-point value is unaffected by this bookkeeping.
        double veh_cv = 0.0, veh_wv = 0.0;
        for (const auto& r : sol.truck_routes[t]) {
            sum                += r->working_time();
            capacity_violation += r->capacity_violation() / cfg.truck.capacity;
            waiting_time_violation += r->waiting_time_violation();
            veh_cv += r->capacity_violation() / cfg.truck.capacity;
            veh_wv += r->waiting_time_violation();
        }
        sol.truck_working_time[t]       = sum;
        sol.truck_capacity_violation[t] = veh_cv;
        sol.truck_waiting_violation[t]  = veh_wv;
        working_time = std::max(working_time, sum);
    }

    for (size_t d = 0; d < sol.drone_routes.size(); ++d) {
        double sum = 0.0;
        double veh_ev = 0.0, veh_cv = 0.0, veh_wv = 0.0, veh_fv = 0.0;
        for (const auto& r : sol.drone_routes[d]) {
            sum                  += r->working_time();
            energy_violation     += r->energy_violation;
            capacity_violation   += r->capacity_violation() / cfg.drone.capacity();
            waiting_time_violation += r->waiting_time_violation();
            fixed_time_violation += r->fixed_time_violation;
            veh_ev += r->energy_violation;
            veh_cv += r->capacity_violation() / cfg.drone.capacity();
            veh_wv += r->waiting_time_violation();
            veh_fv += r->fixed_time_violation;
        }
        sol.drone_working_time[d]       = sum;
        sol.drone_energy_violation[d]   = veh_ev;
        sol.drone_capacity_violation[d] = veh_cv;
        sol.drone_waiting_violation[d]  = veh_wv;
        sol.drone_fixed_violation[d]    = veh_fv;
        working_time = std::max(working_time, sum);
    }

    energy_violation       /= cfg.drone.battery();
    waiting_time_violation /= cfg.waiting_time_limit;
    fixed_time_violation   /= cfg.drone.fixed_time();  // x/infinity = 0 for Unlimited

    sol.working_time           = working_time;
    sol.energy_violation       = energy_violation;
    sol.capacity_violation     = capacity_violation;
    sol.waiting_time_violation = waiting_time_violation;
    sol.fixed_time_violation   = fixed_time_violation;
    sol.feasible = (energy_violation       == 0.0 &&
                    capacity_violation     == 0.0 &&
                    waiting_time_violation == 0.0 &&
                    fixed_time_violation   == 0.0);
    return sol;
}

// -----------------------------------------------------------------------
// cost()
// -----------------------------------------------------------------------
double Solution::cost() const {
    double p = 1.0
        + penalty::get(0) * energy_violation
        + penalty::get(1) * capacity_violation
        + penalty::get(2) * waiting_time_violation
        + penalty::get(3) * fixed_time_violation;
    const double exp = global_config().penalty_exponent;
    return working_time * (exp == 0.5 ? std::sqrt(p) : std::pow(p, exp));
}

// -----------------------------------------------------------------------
// hamming_distance
// -----------------------------------------------------------------------
size_t Solution::hamming_distance(const Solution& other) const {
    const Config& cfg = global_config();
    auto fill = [&](const Solution& s, std::vector<size_t>& repr) {
        for (const auto& routes : s.truck_routes)
            for (const auto& r : routes) {
                const auto& c = r->data().customers;
                for (size_t i = 1; i + 1 < c.size(); ++i) repr[c[i]] = c[i+1];
            }
        for (const auto& routes : s.drone_routes)
            for (const auto& r : routes) {
                const auto& c = r->data().customers;
                for (size_t i = 1; i + 1 < c.size(); ++i) repr[c[i]] = c[i+1];
            }
    };
    std::vector<size_t> a(cfg.customers_count+1, 0), b(cfg.customers_count+1, 0);
    fill(*this, a); fill(other, b);
    size_t count = 0;
    for (size_t i = 0; i <= cfg.customers_count; ++i) if (a[i] != b[i]) ++count;
    return count;
}

// -----------------------------------------------------------------------
// verify
// -----------------------------------------------------------------------
void Solution::verify() const {
    const Config& cfg = global_config();
    std::vector<bool> served(cfg.customers_count+1, false);
    served[0] = true;

    auto check = [&](const auto& vehicle_routes, bool sc, bool sr) {
        for (const auto& routes : vehicle_routes) {
            if (sr && routes.size() > 1)
                throw std::runtime_error("Vehicle has more than one route");
            for (const auto& r : routes) {
                const auto& c = r->data().customers;
                if (sc && c.size() != 3)
                    throw std::runtime_error("Route has more than one customer");
                if (c.front() != 0 || c.back() != 0)
                    throw std::runtime_error("Invalid route endpoints");
                for (size_t i = 1; i+1 < c.size(); ++i) {
                    if (served[c[i]]) throw std::runtime_error(
                        "Customer " + std::to_string(c[i]) + " served twice");
                    served[c[i]] = true;
                }
            }
        }
    };
    check(truck_routes, TruckRoute::single_customer(), TruckRoute::single_route());
    check(drone_routes, DroneRoute::single_customer(), DroneRoute::single_route());

    for (size_t i = 0; i <= cfg.customers_count; ++i)
        if (!served[i]) throw std::runtime_error(
            "Customer " + std::to_string(i) + " not served");
}

// -----------------------------------------------------------------------
// JSON serialization
// -----------------------------------------------------------------------
nlohmann::json Solution::to_json() const {
    nlohmann::json j;
    // truck_routes: array of array of array of size_t
    auto route_arr = [](const auto& vr) {
        nlohmann::json a = nlohmann::json::array();
        for (const auto& routes : vr) {
            nlohmann::json va = nlohmann::json::array();
            for (const auto& r : routes)
                va.push_back(r->data().customers);
            a.push_back(va);
        }
        return a;
    };
    j["truck_routes"]          = route_arr(truck_routes);
    j["drone_routes"]          = route_arr(drone_routes);
    j["truck_working_time"]    = truck_working_time;
    j["drone_working_time"]    = drone_working_time;
    j["working_time"]          = working_time;
    j["energy_violation"]      = energy_violation;
    j["capacity_violation"]    = capacity_violation;
    j["waiting_time_violation"]= waiting_time_violation;
    j["fixed_time_violation"]  = fixed_time_violation;
    j["feasible"]              = feasible;
    return j;
}

// -----------------------------------------------------------------------
// initialize() – copied from the master-slave tabu search; the RNG seed is
// a parameter so that the GA can build several different solutions.
// -----------------------------------------------------------------------
Solution Solution::initialize(uint64_t seed)
{
    const Config& cfg = global_config();

    std::vector<size_t> index;
    index.reserve(cfg.customers_count);
    for (size_t i = 1; i <= cfg.customers_count; ++i) index.push_back(i);

    auto clusters = clusterize(index, cfg.trucks_count);

    std::vector<std::vector<LocalRc<TruckRoute>>> truck_routes(cfg.trucks_count);
    std::vector<std::vector<LocalRc<DroneRoute>>> drone_routes(cfg.trucks_count);

    std::vector<size_t> clusters_mapping(cfg.customers_count+1, 0);
    for (size_t ci = 0; ci < clusters.size(); ++ci)
        for (size_t c : clusters[ci]) clusters_mapping[c] = ci;

    // Check feasibility helper
    auto feasible_check = [](const std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
                              const std::vector<std::vector<LocalRc<DroneRoute>>>& dr) {
        return Solution::make(tr, dr).feasible;
    };

    // Determine which customers are servable by truck / drone
    std::vector<bool> truckable(cfg.customers_count+1, false);
    std::vector<bool> dronable_local(cfg.customers_count+1, false);
    truckable[0]       = true;
    dronable_local[0]  = true;

    if (cfg.trucks_count > 0) {
        for (size_t c = 1; c <= cfg.customers_count; ++c) {
            truck_routes[0].push_back(TruckRoute::make_single(c));
            truckable[c] = feasible_check(truck_routes, drone_routes);
            truck_routes[0].pop_back();
        }
    }
    if (cfg.drones_count > 0) {
        for (size_t c = 1; c <= cfg.customers_count; ++c) {
            if (cfg.dronable[c]) {
                drone_routes[0].push_back(DroneRoute::make_single(c));
                dronable_local[c] = feasible_check(truck_routes, drone_routes);
                drone_routes[0].pop_back();
            }
        }
    }

    for (size_t c = 1; c <= cfg.customers_count; ++c)
        if (!truckable[c] && !dronable_local[c])
            throw std::runtime_error("Customer " + std::to_string(c) +
                " cannot be served by neither trucks nor drones");

    // Priority-queue state
    struct State {
        double working_time;
        size_t vehicle;
        size_t parent;
        size_t index;
        bool   is_truck;
        bool operator>(const State& o) const {
            return working_time > o.working_time;
        }
    };

    // Min-heap (pop smallest working_time first)
    std::priority_queue<State, std::vector<State>, std::greater<State>> queue;

    // RNG
    std::mt19937_64 rng(seed);

    for (size_t ci = 0; ci < clusters.size(); ++ci) {
        auto& cluster = clusters[ci];
        if (cluster.empty()) continue;

        // Shuffle and push first truckable as truck candidate
        std::shuffle(cluster.begin(), cluster.end(), rng);
        for (size_t c : cluster) {
            if (truckable[c]) {
                queue.push({0.0, ci, 0, cluster.front(), true});
                break;
            }
        }

        // Sort by drone distance from depot, push first dronable
        std::sort(cluster.begin(), cluster.end(), [&](size_t a, size_t b){
            return cfg.drone_distances[0][a] < cfg.drone_distances[0][b];
        });
        for (size_t c : cluster) {
            if (dronable_local[c]) {
                queue.push({0.0, ci, 0, cluster[0], false});
                break;
            }
        }
    }

    std::set<size_t> global_set;
    for (size_t i = 1; i <= cfg.customers_count; ++i) global_set.insert(i);

    // Helper: push next truck candidate
    auto truck_next = [&](size_t parent, size_t vehicle) {
        double best_d = std::numeric_limits<double>::infinity();
        size_t best_c = 0;
        const auto& cl = clusters[clusters_mapping[parent]];
        for (size_t c : cl) {
            if (truckable[c] && global_set.count(c) &&
                cfg.truck_distances[parent][c] < best_d) {
                best_d = cfg.truck_distances[parent][c];
                best_c = c;
            }
        }
        if (best_c == 0) {
            for (size_t c : global_set) {
                if (truckable[c] && cfg.truck_distances[parent][c] < best_d) {
                    best_d = cfg.truck_distances[parent][c];
                    best_c = c;
                }
            }
        }
        if (best_c != 0) {
            double wt = Solution::make(truck_routes, drone_routes).truck_working_time[vehicle];
            queue.push({wt, vehicle, parent, best_c, true});
        }
    };

    auto drone_next = [&](size_t parent, size_t vehicle) {
        double best_d = std::numeric_limits<double>::infinity();
        size_t best_c = 0;
        const auto& cl = clusters[clusters_mapping[parent]];
        for (size_t c : cl) {
            if (dronable_local[c] && global_set.count(c) &&
                cfg.drone_distances[parent][c] < best_d) {
                best_d = cfg.drone_distances[parent][c];
                best_c = c;
            }
        }
        if (best_c == 0) {
            for (size_t c : global_set) {
                if (dronable_local[c] && cfg.drone_distances[parent][c] < best_d) {
                    best_d = cfg.drone_distances[parent][c];
                    best_c = c;
                }
            }
        }
        if (best_c != 0) {
            double wt = Solution::make(truck_routes, drone_routes).drone_working_time[vehicle];
            queue.push({wt, vehicle, parent, best_c, false});
        }
    };

    while (!global_set.empty()) {
        if (queue.empty())
            throw std::runtime_error(
                "Cannot construct initial solution – unservable customers remain");

        State packed = queue.top(); queue.pop();
        size_t v     = packed.vehicle;

        // Is the index still in the cluster?
        auto& cl = clusters[clusters_mapping[packed.index]];
        auto  it = std::find(cl.begin(), cl.end(), packed.index);

        if (it != cl.end()) {
            if (packed.is_truck) {
                if (packed.parent == 0)
                    truck_routes[v].push_back(TruckRoute::make_single(packed.index));
                else {
                    auto& rt = truck_routes[v].back();
                    rt = rt->route_push(packed.index);
                }
            } else {
                if (packed.parent == 0)
                    drone_routes[v].push_back(DroneRoute::make_single(packed.index));
                else {
                    auto& rt = drone_routes[v].back();
                    rt = rt->route_push(packed.index);
                }
            }

            if (feasible_check(truck_routes, drone_routes)) {
                cl.erase(it);
                global_set.erase(packed.index);

                if (packed.is_truck)
                    truck_next(packed.index, v);
                else
                    drone_next(cfg.single_drone_route ? 0 : packed.index, v);
            } else {
                // Undo
                if (packed.is_truck) {
                    if (packed.parent == 0)
                        truck_routes[v].pop_back();
                    else {
                        auto& rt = truck_routes[v].back();
                        rt = rt->route_pop();
                    }
                    if (!cfg.single_truck_route)
                        truck_next(0, v);
                } else {
                    if (packed.parent == 0)
                        drone_routes[v].pop_back();
                    else {
                        auto& rt = drone_routes[v].back();
                        rt = rt->route_pop();
                    }
                    drone_next(0, v);
                }
            }
        } else {
            // index no longer in cluster (already assigned)
            if (packed.is_truck)
                truck_next(packed.parent, v);
            else
                drone_next(cfg.single_drone_route ? 0 : packed.parent, v);
        }
    }

    // Resize drone routes to cfg.drones_count
    if (cfg.drones_count > 0) {
        std::vector<LocalRc<DroneRoute>> all_routes;
        for (auto& v : drone_routes)
            for (auto& r : v) all_routes.push_back(r);
        std::sort(all_routes.begin(), all_routes.end(),
                  [](const auto& a, const auto& b){
                      return a->working_time() > b->working_time();
                  });

        drone_routes.assign(cfg.drones_count, {});
        std::vector<double> wt(cfg.drones_count, 0.0);
        for (const auto& r : all_routes) {
            size_t best = std::min_element(wt.begin(), wt.end()) - wt.begin();
            drone_routes[best].push_back(r);
            wt[best] += r->working_time();
        }
    } else {
        drone_routes.clear();
    }

    return Solution::make(std::move(truck_routes), std::move(drone_routes));
}
