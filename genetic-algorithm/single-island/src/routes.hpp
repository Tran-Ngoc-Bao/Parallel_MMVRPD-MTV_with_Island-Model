#pragma once
#include <vector>
#include <memory>
#include "rc.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include "config.hpp"

// Forward declarations
class TruckRoute;
class DroneRoute;
struct Solution;

// -----------------------------------------------------------------------
// RouteData
// -----------------------------------------------------------------------
struct RouteData {
    std::vector<size_t> customers;
    double distance = 0.0;
    double weight   = 0.0;

    static RouteData construct(std::vector<size_t> customers,
                               const std::vector<std::vector<double>>& distances)
    {
        assert(customers.front() == 0);
        assert(customers.back()  == 0);
        assert(customers.size()  >= 3);
        RouteData d;
        d.customers = std::move(customers);
        for (size_t i = 0; i + 1 < d.customers.size(); ++i) {
            d.distance += distances[d.customers[i]][d.customers[i+1]];
            d.weight   += global_config().demands[d.customers[i]];
        }
        return d;
    }
};

// -----------------------------------------------------------------------
// Abstract base
// -----------------------------------------------------------------------
class RouteBase {
public:
    virtual const RouteData& data() const = 0;
    virtual double working_time() const = 0;
    virtual double capacity_violation() const = 0;
    virtual double waiting_time_violation() const = 0;
    virtual ~RouteBase() = default;
};

// -----------------------------------------------------------------------
// RouteHelper – static dispatch per type (specialised after class defs)
// -----------------------------------------------------------------------
template<typename T> struct RouteHelper;

// -----------------------------------------------------------------------
// CRTP Route base – provides push/pop
// -----------------------------------------------------------------------
template<typename Derived>
class Route : public RouteBase {
public:
    // ---- Shared manipulators ----
    LocalRc<Derived> route_push(size_t customer) const {
        const auto& custs = data().customers;
        auto nc = custs;
        nc.insert(nc.end() - 1, customer);
        return RouteHelper<Derived>::make(std::move(nc));
    }

    LocalRc<Derived> route_pop() const {
        const auto& custs = data().customers;
        auto nc = custs;
        nc.erase(nc.end() - 2);
        return RouteHelper<Derived>::make(std::move(nc));
    }
};

// -----------------------------------------------------------------------
// TruckRoute
// -----------------------------------------------------------------------
class TruckRoute : public Route<TruckRoute> {
public:
    RouteData _data;
    double _working_time          = 0;
    double _capacity_violation    = 0;
    double _waiting_time_violation= 0;

    const RouteData& data()               const override { return _data; }
    double working_time()                 const override { return _working_time; }
    double capacity_violation()           const override { return _capacity_violation; }
    double waiting_time_violation()       const override { return _waiting_time_violation; }

    static LocalRc<TruckRoute> make(std::vector<size_t> customers);
    static LocalRc<TruckRoute> make_single(size_t customer) {
        return make({0, customer, 0});
    }

    static bool servable(size_t /*c*/) { return true; }
    static bool single_customer()      { return false; }
    static bool single_route()         { return global_config().single_truck_route; }

    static std::vector<std::vector<LocalRc<TruckRoute>>>& get_routes_mut(
        std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
        std::vector<std::vector<LocalRc<DroneRoute>>>& /*dr*/) { return tr; }

    static const std::vector<std::vector<LocalRc<TruckRoute>>>& get_routes(
        const std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
        const std::vector<std::vector<LocalRc<DroneRoute>>>& /*dr*/) { return tr; }

    // Passkey for LocalRc construction — only TruckRoute can create PrivateTag
    struct PrivateTag { friend class TruckRoute; private: PrivateTag() = default; };
    explicit TruckRoute(PrivateTag) {}

private:
    TruckRoute() = default;
    friend class Route<TruckRoute>;
};

// -----------------------------------------------------------------------
// DroneRoute
// -----------------------------------------------------------------------
class DroneRoute : public Route<DroneRoute> {
public:
    RouteData _data;
    double _working_time          = 0;
    double _capacity_violation    = 0;
    double _waiting_time_violation= 0;
    double energy_violation       = 0;
    double fixed_time_violation   = 0;

    const RouteData& data()               const override { return _data; }
    double working_time()                 const override { return _working_time; }
    double capacity_violation()           const override { return _capacity_violation; }
    double waiting_time_violation()       const override { return _waiting_time_violation; }

    static LocalRc<DroneRoute> make(std::vector<size_t> customers);
    static LocalRc<DroneRoute> make_single(size_t customer) {
        return make({0, customer, 0});
    }

    static bool servable(size_t c)  { return global_config().dronable[c]; }
    static bool single_customer()   { return global_config().single_drone_route; }
    static bool single_route()      { return false; }

    static std::vector<std::vector<LocalRc<DroneRoute>>>& get_routes_mut(
        std::vector<std::vector<LocalRc<TruckRoute>>>& /*tr*/,
        std::vector<std::vector<LocalRc<DroneRoute>>>& dr) { return dr; }

    static const std::vector<std::vector<LocalRc<DroneRoute>>>& get_routes(
        const std::vector<std::vector<LocalRc<TruckRoute>>>& /*tr*/,
        const std::vector<std::vector<LocalRc<DroneRoute>>>& dr) { return dr; }

    // Passkey for LocalRc construction — only DroneRoute can create PrivateTag
    struct PrivateTag { friend class DroneRoute; private: PrivateTag() = default; };
    explicit DroneRoute(PrivateTag) {}

private:
    DroneRoute() = default;
    friend class Route<DroneRoute>;
};

// -----------------------------------------------------------------------
// RouteHelper specialisations
// -----------------------------------------------------------------------
template<> struct RouteHelper<TruckRoute> {
    static bool servable(size_t c)  { return TruckRoute::servable(c); }
    static bool single_customer()   { return TruckRoute::single_customer(); }
    static bool single_route()      { return TruckRoute::single_route(); }
    static LocalRc<TruckRoute> make(std::vector<size_t> c) {
        return TruckRoute::make(std::move(c));
    }
    static std::vector<std::vector<LocalRc<TruckRoute>>>& get_routes_mut(
        std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
        std::vector<std::vector<LocalRc<DroneRoute>>>& dr) {
        return TruckRoute::get_routes_mut(tr, dr);
    }
    static const std::vector<std::vector<LocalRc<TruckRoute>>>& get_routes(
        const std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
        const std::vector<std::vector<LocalRc<DroneRoute>>>& dr) {
        return TruckRoute::get_routes(tr, dr);
    }
};

template<> struct RouteHelper<DroneRoute> {
    static bool servable(size_t c)  { return DroneRoute::servable(c); }
    static bool single_customer()   { return DroneRoute::single_customer(); }
    static bool single_route()      { return DroneRoute::single_route(); }
    static LocalRc<DroneRoute> make(std::vector<size_t> c) {
        return DroneRoute::make(std::move(c));
    }
    static std::vector<std::vector<LocalRc<DroneRoute>>>& get_routes_mut(
        std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
        std::vector<std::vector<LocalRc<DroneRoute>>>& dr) {
        return DroneRoute::get_routes_mut(tr, dr);
    }
    static const std::vector<std::vector<LocalRc<DroneRoute>>>& get_routes(
        const std::vector<std::vector<LocalRc<TruckRoute>>>& tr,
        const std::vector<std::vector<LocalRc<DroneRoute>>>& dr) {
        return DroneRoute::get_routes(tr, dr);
    }
};

using TruckRoutes = std::vector<std::vector<LocalRc<TruckRoute>>>;
using DroneRoutes = std::vector<std::vector<LocalRc<DroneRoute>>>;

// Same formula as Solution::cost(), from already-normalized totals.
double penalized_cost(double working_time, double energy_violation,
                      double capacity_violation, double waiting_time_violation,
                      double fixed_time_violation);

// Per-vehicle sums of route working time / violations. `cv` is normalized
// by capacity per route (as in Solution::make); the others are raw sums.
struct VehicleStats {
    double wt = 0, cv = 0, wv = 0, ev = 0, fv = 0;

    VehicleStats& operator+=(const VehicleStats& o) {
        wt += o.wt; cv += o.cv; wv += o.wv; ev += o.ev; fv += o.fv;
        return *this;
    }
    VehicleStats& operator-=(const VehicleStats& o) {
        wt -= o.wt; cv -= o.cv; wv -= o.wv; ev -= o.ev; fv -= o.fv;
        return *this;
    }
};

VehicleStats stats_of(const TruckRoute& r);
VehicleStats stats_of(const DroneRoute& r);

// True when the route has no violation at all.
bool route_feasible(const TruckRoute& r);
bool route_feasible(const DroneRoute& r);

// Penalized cost of a fleet given per-vehicle stats (same as Solution::cost()).
double fleet_cost(const std::vector<VehicleStats>& trucks,
                  const std::vector<VehicleStats>& drones);
