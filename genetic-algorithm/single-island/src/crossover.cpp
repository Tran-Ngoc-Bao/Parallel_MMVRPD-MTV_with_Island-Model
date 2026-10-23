#include "crossover.hpp"
#include "config.hpp"
#include <algorithm>

namespace crossover {

std::vector<std::size_t> ordered(const std::vector<std::size_t>& p1,
                                 const std::vector<std::size_t>& p2,
                                 std::mt19937_64& rng)
{
    const std::size_t n = p1.size();
    if (n < 2) return p1;

    std::uniform_int_distribution<std::size_t> pick(0, n - 1);
    std::size_t first = pick(rng), second = pick(rng);
    if (first > second) std::swap(first, second);

    std::vector<std::size_t> child(n);
    std::vector<bool> used(global_config().customers_count + 1, false);
    for (std::size_t k = first; k <= second; ++k) {
        child[k] = p1[k];
        used[p1[k]] = true;
    }

    std::size_t pos = (second + 1) % n;
    for (std::size_t t = 0; t < n; ++t) {
        std::size_t c = p2[(second + 1 + t) % n];
        if (used[c]) continue;
        child[pos] = c;
        pos = (pos + 1) % n;
    }
    return child;
}

} // namespace crossover
