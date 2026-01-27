#include <map>
#include <ranges>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/common/unused.h"

namespace {

void using_concat(const std::map<int, int>& data) {
  fmt::print("\nConcatenation\n");
  drake::test::LimitMalloc guard({max_num_allocations : -1});
  auto it = data.begin();
  std::string result = fmt::format("{}-{}", it->first, it->second);
  for (++it; it != data.end(); ++it) {
    result += fmt::format(" | {}-{}", it->first, it->second);
  }
  fmt::print("Allocations: {}\n", guard.num_allocations());
  fmt::print("Result length: {}\n", result.size());
}

void using_append(const std::map<int, int>& data) {
  fmt::print("\nAppend\n");
  drake::test::LimitMalloc guard({max_num_allocations : -1});
  auto it = data.begin();
  std::string result = fmt::format("{}-{}", it->first, it->second);
  for (++it; it != data.end(); ++it) {
    result.append(fmt::format(" | {}-{}", it->first, it->second));
  }
  fmt::print("Allocations: {}\n", guard.num_allocations());
  fmt::print("Result length: {}\n", result.size());
}

void using_fmt(const std::map<int, int>& data) {
  fmt::print("\nfmt join\n");


  drake::test::LimitMalloc guard({max_num_allocations : -1});
  auto view = data | std::views::transform([](const auto& pair) {
                return fmt::format("{}-{}", pair.first, pair.second);
              });
//   const std::string result = fmt::to_string(fmt::join(view, " | "));
  
  drake::unused(view);
  
  fmt::print("Allocations: {}\n", guard.num_allocations());
//   fmt::print("Result length: {}\n", result.size());
}

}  // namespace

int main() {
  std::map<int, int> data;
  for (int i = 0; i < 10; ++i) {
    data.emplace(i, i * 10);
  }
  using_concat(data);
  using_append(data);
  using_fmt(data);

  // Proof that the adapter returned by transform doesn't store the transformed
  // results. We print the formatting message each time we pass through.
  auto view = data | std::views::transform([](const auto& pair) -> std::string {
                fmt::print("Formatting {}, {}\n", pair.first, pair.second);
                return fmt::format("{}-{}", pair.first, pair.second);
              });
  for (const auto& v : view )
    fmt::print("Value: {}\n", v);
  for (const auto& v : view )
    fmt::print("Value: {}\n", v);
  return 1;
}
