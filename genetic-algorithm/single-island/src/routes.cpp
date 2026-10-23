#include "routes.hpp"
#include "solutions.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

// -----------------------------------------------------------------------
// TruckRoute::make
// -----------------------------------------------------------------------
LocalRc<TruckRoute> TruckRoute::make(std::vector<size_t> customers)
{
    const Config& cfg = global_config();
    auto r = LocalRc<TruckRoute>::make(TruckRoute::PrivateTag{});
    r->_data = RouteData::construct(std::move(customers), cfg.truck_distances);

    double speed = cfg.truck.speed;
    r->_working_time       = r->_data.distance / speed;
    r->_capacity_violation = std::max(0.0, r->_data.weight - cfg.truck.capacity);

    // Waiting time violation
    double wtv    = 0.0;
    double accum  = 0.0;
    const auto& d = cfg.truck_distances;
    const auto& c = r->_data.customers;
    for (size_t i = 1; i + 1 < c.size(); ++i) {
        accum += d[c[i-1]][c[i]] / speed;
        wtv   += std::max(0.0, r->_working_time - accum - cfg.waiting_time_limit);
    }
    r->_waiting_time_violation = wtv;
    return r;
}

// -----------------------------------------------------------------------
// DroneRoute::make
// -----------------------------------------------------------------------
LocalRc<DroneRoute> DroneRoute::make(std::vector<size_t> customers)
{
    const Config& cfg = global_config();
    auto r = LocalRc<DroneRoute>::make(DroneRoute::PrivateTag{});
    r->_data = RouteData::construct(std::move(customers), cfg.drone_distances);

    const DroneConfig& drone = cfg.drone;
    const auto& c   = r->_data.customers;
    const auto& dist= cfg.drone_distances;

    double takeoff  = drone.takeoff_time();
    double landing  = drone.landing_time();

    r->_working_time = (takeoff + landing) * static_cast<double>(c.size() - 1)
                       + drone.cruise_time(r->_data.distance);
    r->_capacity_violation = std::max(0.0, r->_data.weight - drone.capacity());

    double time   = 0.0;
    double energy = 0.0;
    double weight = 0.0;
    double wtv    = 0.0;

    for (size_t i = 0; i + 1 < c.size(); ++i) {
        double cruise = drone.cruise_time(dist[c[i]][c[i+1]]);
        time   += takeoff + cruise + landing;
        energy += drone.landing_power(weight) * landing
                + drone.takeoff_power(weight) * takeoff
                + drone.cruise_power(weight)  * cruise;
        weight += cfg.demands[c[i]];
        wtv    += std::max(0.0, r->_working_time - time - cfg.waiting_time_limit);
    }

    r->energy_violation       = std::max(0.0, energy - drone.battery());
    r->fixed_time_violation   = std::max(0.0, r->_working_time - drone.fixed_time());
    r->_waiting_time_violation= wtv;
    return r;
}

double penalized_cost(double working_time, double energy_violation,
                      double capacity_violation, double waiting_time_violation,
                      double fixed_time_violation)
{
    double p = 1.0
        + penalty::get(0) * energy_violation
        + penalty::get(1) * capacity_violation
        + penalty::get(2) * waiting_time_violation
        + penalty::get(3) * fixed_time_violation;
    const double exp = global_config().penalty_exponent;
    return working_time * (exp == 0.5 ? std::sqrt(p) : std::pow(p, exp));
}

VehicleStats stats_of(const TruckRoute& r) {
    const Config& cfg = global_config();
    return {r.working_time(), r.capacity_violation() / cfg.truck.capacity,
            r.waiting_time_violation(), 0.0, 0.0};
}

VehicleStats stats_of(const DroneRoute& r) {
    const Config& cfg = global_config();
    return {r.working_time(), r.capacity_violation() / cfg.drone.capacity(),
            r.waiting_time_violation(), r.energy_violation, r.fixed_time_violation};
}

bool route_feasible(const TruckRoute& r) {
    return r.capacity_violation() == 0.0 && r.waiting_time_violation() == 0.0;
}

bool route_feasible(const DroneRoute& r) {
    return r.capacity_violation() == 0.0 && r.waiting_time_violation() == 0.0 &&
           r.energy_violation == 0.0 && r.fixed_time_violation == 0.0;
}

double fleet_cost(const std::vector<VehicleStats>& trucks,
                  const std::vector<VehicleStats>& drones)
{
    const Config& cfg = global_config();
    double wt = 0, cv = 0, wv = 0, ev = 0, fv = 0;
    auto acc = [&](const VehicleStats& s) {
        wt = std::max(wt, s.wt); cv += s.cv; wv += s.wv; ev += s.ev; fv += s.fv;
    };
    for (const auto& s : trucks) acc(s);
    for (const auto& s : drones) acc(s);
    return penalized_cost(wt, ev / cfg.drone.battery(), cv,
                          wv / cfg.waiting_time_limit, fv / cfg.drone.fixed_time());
}
