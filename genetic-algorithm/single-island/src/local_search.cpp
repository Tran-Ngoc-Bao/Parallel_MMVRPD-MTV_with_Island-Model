#include "local_search.hpp"
#include <algorithm>
#include <numeric>
#include <limits>

namespace local_search {

namespace {

constexpr double      EPS      = 1e-6;
constexpr std::size_t NEW_TRIP = std::numeric_limits<std::size_t>::max();

using Seq = std::vector<std::size_t>;   // trip customers, without the depot

struct Trip {
    bool         is_truck = true;
    std::size_t  vehicle  = 0;
    Seq          seq;
    LocalRc<TruckRoute> truck;
    LocalRc<DroneRoute> drone;
    VehicleStats stats;
};

// New content for one trip; an empty seq removes the trip.
struct Change {
    std::size_t trip     = NEW_TRIP;
    Seq         seq;
    bool        is_truck = true;   // only for NEW_TRIP
    std::size_t vehicle  = 0;      // only for NEW_TRIP
};

// Customers sorted by truck distance from each customer (granular
// neighbourhood), truncated to --ls-neighbors.
const std::vector<Seq>& neighbors()
{
    static const std::vector<Seq> nb = [] {
        const Config& cfg = global_config();
        const std::size_t n = cfg.customers_count;
        std::size_t k = cfg.ls_neighbors == 0 ? n - 1 : std::min(cfg.ls_neighbors, n - 1);
        std::vector<Seq> res(n + 1);
        for (std::size_t u = 1; u <= n; ++u) {
            Seq& list = res[u];
            for (std::size_t v = 1; v <= n; ++v) if (v != u) list.push_back(v);
            std::partial_sort(list.begin(), list.begin() + static_cast<std::ptrdiff_t>(k), list.end(),
                              [&](std::size_t a, std::size_t b) {
                                  return cfg.truck_distances[u][a] < cfg.truck_distances[u][b];
                              });
            list.resize(k);
        }
        return res;
    }();
    return nb;
}

Seq concat(Seq a, const Seq& b) { a.insert(a.end(), b.begin(), b.end()); return a; }
Seq reversed(Seq a) { std::reverse(a.begin(), a.end()); return a; }
Seq slice(const Seq& s, std::size_t from, std::size_t to) {
    return Seq(s.begin() + static_cast<std::ptrdiff_t>(from), s.begin() + static_cast<std::ptrdiff_t>(to));
}

Seq erase_values(const Seq& s, std::size_t a, std::size_t b = 0) {
    Seq out;
    out.reserve(s.size());
    for (std::size_t c : s) if (c != a && c != b) out.push_back(c);
    return out;
}

Seq insert_after(Seq s, std::size_t v, const Seq& items) {
    auto it = std::find(s.begin(), s.end(), v) + 1;
    s.insert(it, items.begin(), items.end());
    return s;
}

class Search {
public:
    Search(const Solution& s, std::size_t& evaluations)
        : ts_(s.truck_routes.size()), ds_(s.drone_routes.size()),
          evals_(evaluations)
    {
        for (std::size_t t = 0; t < s.truck_routes.size(); ++t)
            for (const auto& r : s.truck_routes[t])
                trips_.push_back(make_trip(true, t, r, {}));
        for (std::size_t d = 0; d < s.drone_routes.size(); ++d)
            for (const auto& r : s.drone_routes[d])
                trips_.push_back(make_trip(false, d, {}, r));
        refresh();
    }

    // Tries M1..M9 for (u, v); applies the first improving one.
    bool try_moves(std::size_t u, std::size_t v);

    // Relocates u into a new trip of the least-loaded truck / drone.
    bool try_new_trip(std::size_t u);

    Solution build() const
    {
        TruckRoutes tr(ts_.size());
        DroneRoutes dr(ds_.size());
        for (const auto& t : trips_) {
            if (t.is_truck) tr[t.vehicle].push_back(t.truck);
            else            dr[t.vehicle].push_back(t.drone);
        }
        return Solution::make(std::move(tr), std::move(dr));
    }

private:
    std::vector<Trip> trips_;
    std::vector<VehicleStats> ts_, ds_;
    std::vector<std::size_t> trip_of_, index_of_;
    double cost_  = 0;
    double total_ = 0;
    std::size_t& evals_;

    static Trip make_trip(bool is_truck, std::size_t v,
                          LocalRc<TruckRoute> tr, LocalRc<DroneRoute> dr)
    {
        const auto& c = is_truck ? tr->data().customers : dr->data().customers;
        Trip t{is_truck, v, Seq(c.begin() + 1, c.end() - 1), tr, dr, {}};
        t.stats = is_truck ? stats_of(*tr) : stats_of(*dr);
        return t;
    }

    // Recomputes vehicle stats, objective and customer positions.
    void refresh()
    {
        std::fill(ts_.begin(), ts_.end(), VehicleStats{});
        std::fill(ds_.begin(), ds_.end(), VehicleStats{});
        total_ = 0;
        trip_of_.assign(global_config().customers_count + 1, 0);
        index_of_.assign(global_config().customers_count + 1, 0);
        for (std::size_t k = 0; k < trips_.size(); ++k) {
            const Trip& t = trips_[k];
            (t.is_truck ? ts_ : ds_)[t.vehicle] += t.stats;
            total_ += t.stats.wt;
            for (std::size_t i = 0; i < t.seq.size(); ++i) {
                trip_of_[t.seq[i]]  = k;
                index_of_[t.seq[i]] = i;
            }
        }
        cost_ = fleet_cost(ts_, ds_);
    }

    bool try_changes(std::vector<Change> changes);
};

bool Search::try_changes(std::vector<Change> changes)
{
    const Config& cfg = global_config();
    auto ts = ts_;
    auto ds = ds_;
    double total = total_;
    std::vector<Trip> built(changes.size());

    for (std::size_t k = 0; k < changes.size(); ++k) {
        Change& ch = changes[k];
        if (ch.trip != NEW_TRIP) {
            const Trip& old = trips_[ch.trip];
            ch.is_truck = old.is_truck;
            ch.vehicle  = old.vehicle;
            (ch.is_truck ? ts : ds)[ch.vehicle] -= old.stats;
            total -= old.stats.wt;
        }
        if (ch.seq.empty()) continue;

        Seq buf{0};
        buf.insert(buf.end(), ch.seq.begin(), ch.seq.end());
        buf.push_back(0);
        ++evals_;
        if (ch.is_truck) {
            auto r = TruckRoute::make(std::move(buf));
            if (!route_feasible(*r)) return false;
            built[k] = make_trip(true, ch.vehicle, r, {});
        } else {
            if (cfg.single_drone_route && ch.seq.size() > 1) return false;
            for (std::size_t c : ch.seq) if (!cfg.dronable[c]) return false;
            auto r = DroneRoute::make(std::move(buf));
            if (!route_feasible(*r)) return false;
            built[k] = make_trip(false, ch.vehicle, {}, r);
        }
        (ch.is_truck ? ts : ds)[ch.vehicle] += built[k].stats;
        total += built[k].stats.wt;
    }

    double cost = fleet_cost(ts, ds);
    if (!(cost < cost_ - EPS || (cost <= cost_ && total < total_ - EPS))) return false;

    for (std::size_t k = 0; k < changes.size(); ++k) {
        if (changes[k].trip == NEW_TRIP) trips_.push_back(std::move(built[k]));
        else trips_[changes[k].trip] = std::move(built[k]);   // empty seq when removed
    }
    trips_.erase(std::remove_if(trips_.begin(), trips_.end(),
                                [](const Trip& t) { return t.seq.empty(); }),
                 trips_.end());
    refresh();
    return true;
}

bool Search::try_moves(std::size_t u, std::size_t v)
{
    const std::size_t tu = trip_of_[u], iu = index_of_[u];
    const std::size_t tv = trip_of_[v], iv = index_of_[v];
    const Seq A = trips_[tu].seq;
    const Seq B = trips_[tv].seq;
    const std::size_t x = iu + 1 < A.size() ? A[iu + 1] : 0;   // 0 = depot
    const std::size_t y = iv + 1 < B.size() ? B[iv + 1] : 0;

    auto one = [&](Seq s) { return try_changes({{tu, std::move(s)}}); };
    auto two = [&](Seq a, Seq b) { return try_changes({{tu, std::move(a)}, {tv, std::move(b)}}); };

    if (tu != tv) {
        const Seq a_head = slice(A, 0, iu + 1), a_tail = slice(A, iu + 1, A.size());
        const Seq b_head = slice(B, 0, iv + 1), b_tail = slice(B, iv + 1, B.size());
        const Seq a_wo_u  = erase_values(A, u);
        const Seq a_wo_ux = erase_values(A, u, x);
        auto b_with = [&](Seq items, std::size_t skip) {   // replace `skip` items after iv-1
            return concat(concat(slice(B, 0, iv), items), slice(B, iv + skip, B.size()));
        };

        // M1-M3: relocate
        if (two(a_wo_u, insert_after(B, v, {u}))) return true;
        if (x != 0 && two(a_wo_ux, insert_after(B, v, {u, x}))) return true;
        if (x != 0 && two(a_wo_ux, insert_after(B, v, {x, u}))) return true;
        // M4-M6: swap
        {
            Seq a = A; a[iu] = v;
            if (two(a, b_with({u}, 1))) return true;
        }
        if (x != 0) {
            Seq a = concat(concat(slice(A, 0, iu), {v}), slice(A, iu + 2, A.size()));
            if (two(a, b_with({u, x}, 1))) return true;
        }
        if (x != 0 && y != 0) {
            Seq a = concat(concat(slice(A, 0, iu), {v, y}), slice(A, iu + 2, A.size()));
            if (two(a, b_with({u, x}, 2))) return true;
        }
        // M8, M9: 2-Opt*
        if (two(concat(a_head, reversed(b_head)), concat(reversed(a_tail), b_tail))) return true;
        if (two(concat(a_head, b_tail), concat(b_head, a_tail))) return true;
        return false;
    }

    // Same trip. M1-M3: relocate
    if (one(insert_after(erase_values(A, u), v, {u}))) return true;
    if (x != 0 && v != x) {
        if (one(insert_after(erase_values(A, u, x), v, {u, x}))) return true;
        if (one(insert_after(erase_values(A, u, x), v, {x, u}))) return true;
    }
    // M4-M6: swap
    {
        Seq s = A; std::swap(s[iu], s[iv]);
        if (one(s)) return true;
    }
    if (x != 0 && v != x) {
        Seq s;
        for (std::size_t k = 0; k < A.size(); ++k) {
            if (A[k] == u)      { s.push_back(v); ++k; }
            else if (A[k] == v) { s.push_back(u); s.push_back(x); }
            else                  s.push_back(A[k]);
        }
        if (one(s)) return true;
    }
    if (x != 0 && y != 0 && v != x && y != u) {
        Seq s;
        for (std::size_t k = 0; k < A.size(); ++k) {
            if (A[k] == u)      { s.push_back(v); s.push_back(y); ++k; }
            else if (A[k] == v) { s.push_back(u); s.push_back(x); ++k; }
            else                  s.push_back(A[k]);
        }
        if (one(s)) return true;
    }
    // M7: 2-Opt, reverse the part between the two removed edges
    {
        Seq s = A;
        if (iu < iv) std::reverse(s.begin() + static_cast<std::ptrdiff_t>(iu + 1),
                                  s.begin() + static_cast<std::ptrdiff_t>(iv + 1));
        else         std::reverse(s.begin() + static_cast<std::ptrdiff_t>(iv + 1),
                                  s.begin() + static_cast<std::ptrdiff_t>(iu + 1));
        if (s != A && one(s)) return true;
    }
    return false;
}

bool Search::try_new_trip(std::size_t u)
{
    const Config& cfg = global_config();
    const std::size_t tu = trip_of_[u];
    const Seq a = erase_values(trips_[tu].seq, u);

    std::vector<std::size_t> truck_trips(ts_.size(), 0);
    for (const auto& t : trips_) if (t.is_truck) ++truck_trips[t.vehicle];

    std::size_t best_truck = ts_.size();
    for (std::size_t t = 0; t < ts_.size(); ++t) {
        if (cfg.single_truck_route && truck_trips[t] > 0) continue;
        if (best_truck == ts_.size() || ts_[t].wt < ts_[best_truck].wt) best_truck = t;
    }
    if (best_truck < ts_.size() &&
        try_changes({{tu, a}, {NEW_TRIP, {u}, true, best_truck}})) return true;

    if (cfg.dronable[u] && !ds_.empty()) {
        std::size_t best_drone = 0;
        for (std::size_t d = 1; d < ds_.size(); ++d)
            if (ds_[d].wt < ds_[best_drone].wt) best_drone = d;
        if (try_changes({{tu, a}, {NEW_TRIP, {u}, false, best_drone}})) return true;
    }
    return false;
}

} // namespace

Solution improve(const Solution& s, std::mt19937_64& rng, std::size_t& evaluations)
{
    const Config& cfg = global_config();
    Search search(s, evaluations);

    Seq order(cfg.customers_count);
    std::iota(order.begin(), order.end(), std::size_t{1});

    bool improved = true;
    while (improved) {
        improved = false;
        std::shuffle(order.begin(), order.end(), rng);
        for (std::size_t u : order) {
            bool moved = false;
            for (std::size_t v : neighbors()[u])
                if (search.try_moves(u, v)) { moved = true; break; }
            if (!moved) moved = search.try_new_trip(u);
            improved |= moved;
        }
    }
    return search.build();
}

} // namespace local_search
