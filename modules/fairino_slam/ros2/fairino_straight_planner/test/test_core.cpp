#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "fairino_straight_planner/core.hpp"

namespace c = fairino_straight_planner::core;
void require(bool ok, const std::string & why) {if (!ok) {throw std::runtime_error(why);}}
template<class F> void rejects(F fn, const char * why)
{
  bool thrown = false;
  try {fn();} catch (const std::exception &) {thrown = true;}
  require(thrown, why);
}
struct Map
{
  unsigned width{20}, height{20};
  double resolution{1.}, ox{0.}, oy{0.};
  std::vector<unsigned char> cells;
  Map() : cells(width * height, 0) {}
  c::Grid view() const {return {cells.data(), width, height, resolution, ox, oy};}
  void put(unsigned x, unsigned y, unsigned char value) {cells.at(y * width + x) = value;}
};
bool same(c::Point a, c::Point b) {return a.x == b.x && a.y == b.y;}

void free_lines_and_exact_endpoints()
{
  Map m;
  for (const auto & ends : std::vector<std::pair<c::Point, c::Point>>{
      {{.2, .7}, {18.8, 17.4}}, {{2., 2.}, {18., 2.}},
      {{2., 2.}, {2., 18.}}, {{3.123, 5.789}, {3.123, 5.789}}})
  {
    require(c::segment_clear(m.view(), ends.first, ends.second), "free line rejected");
    require(c::segment_clear(m.view(), ends.second, ends.first), "reverse free line rejected");
    const auto dense = c::resample(m.view(), {ends.first, ends.second}, .3);
    require(same(dense.front(), ends.first) && same(dense.back(), ends.second), "endpoint moved");
    for (std::size_t i = 1; i < dense.size(); ++i) {
      require(std::hypot(dense[i].x - dense[i - 1].x, dense[i].y - dense[i - 1].y) <= .3 + 1e-10,
        "resampling spacing exceeded");
    }
  }
  const auto out = c::simplify(m.view(), {{.2, .7}, {2.2, 3.4}, {2.2, 3.4}, {7.9, 6.2}});
  require(out.size() == 2 && same(out.front(), {.2, .7}) && same(out.back(), {7.9, 6.2}),
    "free zigzag was not a direct line");
}

void blocked_unknown_and_boundary_contacts()
{
  Map m;
  for (unsigned char value : {static_cast<unsigned char>(253), static_cast<unsigned char>(254),
      static_cast<unsigned char>(255)})
  {
    m.put(5, 5, value);
    require(!c::segment_clear(m.view(), {.1, .1}, {9.9, 9.9}), "hard blocked cell accepted");
  }
  m.put(5, 5, 252);
  require(c::segment_clear(m.view(), {.1, .1}, {9.9, 9.9}), "soft cost made hard obstacle");
  m.put(5, 5, 0);
  for (c::Point p : std::vector<c::Point>{{0., 3.}, {20., 3.}, {3., 0.}, {3., 20.},
      {-.01, 3.}, {20.01, 3.}, {3., -.01}, {3., 20.01}})
  {
    require(!c::segment_clear(m.view(), {3.5, 3.5}, p), "map boundary contact accepted");
  }
  m.put(1, 0, 254);
  require(!c::segment_clear(m.view(), {.5, .5}, {2.5, 2.5}), "corner-cutting diagonal accepted");
  require(!c::segment_clear(m.view(), {2.5, 2.5}, {.5, .5}), "reverse corner cutting accepted");
  m.put(1, 0, 0);
  m.put(0, 1, 254);
  require(!c::segment_clear(m.view(), {1., .2}, {1., 2.8}), "line along blocked left edge accepted");
  m.put(0, 1, 0);
  m.put(1, 1, 254);
  require(!c::segment_clear(m.view(), {1., .2}, {1., 2.8}), "line along blocked right edge accepted");
  require(!c::segment_clear(m.view(), {1., 1.}, {1., 1.}), "zero-length corner ignored");
}

void fractional_world_and_subcell_collision()
{
  Map m; m.resolution = .05; m.ox = -3.46; m.oy = -3.92;
  const auto point = [&](double x, double y) {return c::Point{m.ox + x * .05, m.oy + y * .05};};
  require(c::segment_clear(m.view(), point(.1, .9), point(17.2, 6.3)), "fractional origin failed");
  m.put(7, 3, 254);
  require(!c::segment_clear(m.view(), point(6.999, 3.001), point(8.001, 3.001)),
    "thin subcell crossing missed");
  m.put(7, 3, 0); m.put(7, 2, 253);
  require(!c::segment_clear(m.view(), point(6.5, 2.5), point(8.5, 4.5)),
    "fractional corner contact missed");
}

void unknown_and_exterior_footprint_clearance()
{
  Map m;
  auto prepared = c::prepare_grid(m.view(), 1.25);
  require(!c::segment_clear(prepared.view(), {1.1, 4.}, {5., 4.}),
    "robot footprint crossed map exterior");
  require(!c::segment_clear(prepared.view(), {1.25, 4.}, {5., 4.}),
    "robot footprint touched map exterior");
  require(c::segment_clear(prepared.view(), {1.5, 4.}, {5., 4.}), "safe inset line rejected");
  m.put(10, 10, 255);
  prepared = c::prepare_grid(m.view(), 1.25);
  require(!c::segment_clear(prepared.view(), {8.9, 10.5}, {8.9, 10.5}),
    "footprint overlapping unknown was accepted");
  require(c::segment_clear(prepared.view(), {7.5, 10.5}, {7.5, 10.5}),
    "distant unknown blocked free point");
  require(prepared.cells[10 * m.width + 10] == 255, "unknown source changed");
  m.put(10, 10, 254);
  prepared = c::prepare_grid(m.view(), 1.25);
  require(prepared.cells[10 * m.width + 8] == 0, "already inflated obstacles were inflated twice");
  m.cells.assign(m.width * m.height, 255);
  m.put(10, 10, 0); m.put(11, 10, 0);
  prepared = c::prepare_grid(m.view(), 1.25);  // Exercises the gather branch.
  require(prepared.cells[10 * m.width + 10] == 255, "unknown gather protection failed");
  const auto small = c::prepare_grid(m.view(), 11.);
  require(!c::segment_clear(small.view(), {10., 10.}, {10., 10.}), "oversized footprint accepted");
}

void greedy_detour_and_invalid_fallback()
{
  Map m;
  for (unsigned y = 0; y < 7; ++y) {m.put(4, y, 254); m.put(5, y, 254);}
  const std::vector<c::Point> raw{{1.5, 1.5}, {1.5, 3.5}, {1.5, 7.5},
    {3.5, 7.5}, {6.5, 7.5}, {8.5, 7.5}, {8.5, 5.5}, {8.5, 1.5}};
  require(!c::segment_clear(m.view(), raw.front(), raw.back()), "wall did not block direct line");
  const auto simple = c::simplify(m.view(), raw);
  require(simple.size() < raw.size() && simple.size() > 2, "detour was not simplified safely");
  require(same(simple.front(), raw.front()) && same(simple.back(), raw.back()), "detour endpoint moved");
  for (std::size_t i = 1; i < simple.size(); ++i) {
    require(c::segment_clear(m.view(), simple[i - 1], simple[i]), "simplified wall crossing");
  }
  const auto dense = c::resample(m.view(), simple, .5);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    require(c::segment_clear(m.view(), dense[i - 1], dense[i]), "dense wall crossing");
  }
  rejects([&] {c::simplify(m.view(), {{1.5, 1.5}, {8.5, 1.5}});}, "unsafe raw edge accepted");
  m.put(2, 2, 255);
  rejects([&] {c::simplify(m.view(), {{1.5, 1.5}, {2.5, 2.5}, {1.5, 7.5}});},
    "unsafe raw point silently accepted");
}


void local_smac_corner_repair()
{
  Map m;
  const std::vector<c::Point> diagonal{{3.5, 3.5}, {4.5, 4.5}};
  m.put(4, 3, 253);
  require(!c::segment_clear(m.view(), diagonal.front(), diagonal.back()),
    "fixture diagonal did not touch a blocked corner");
  const auto repaired = c::repair_grid_edges(m.view(), diagonal);
  require(repaired.size() == 3 && same(repaired[1], {3.5, 4.5}),
    "safe adjacent-grid elbow was not selected");
  require(same(repaired.front(), diagonal.front()) && same(repaired.back(), diagonal.back()),
    "corner repair moved endpoints");
  const auto reverse = c::repair_grid_edges(m.view(), {diagonal.back(), diagonal.front()});
  for (const auto & path : {repaired, reverse}) {
    for (std::size_t i = 1; i < path.size(); ++i) {
      require(c::segment_clear(m.view(), path[i - 1], path[i]), "repair returned unsafe edge");
    }
  }
  m.put(3, 4, 253);
  rejects([&] {c::repair_grid_edges(m.view(), diagonal);},
    "repair accepted two blocked elbows");
  m.put(3, 4, 0); m.put(4, 4, 255);
  rejects([&] {c::repair_grid_edges(m.view(), diagonal);}, "repair accepted blocked endpoint");
  m.put(4, 4, 0);
  rejects([&] {c::repair_grid_edges(m.view(), {{2.5, 2.5}, {5.5, 5.5}});},
    "repair attempted a nonlocal edge through obstacle");
  rejects([&] {c::repair_grid_edges(m.view(), {});}, "repair accepted empty input");

  m.cells.assign(m.width * m.height, 0);
  for (unsigned y = 0; y < 7; ++y) {m.put(4, y, 253); m.put(5, y, 253);}
  const std::vector<c::Point> smac_path{
    {2.5, 5.5}, {3.5, 6.5}, {4.5, 7.5}, {5.5, 7.5}, {6.5, 6.5}, {7.5, 5.5}};
  rejects([&] {c::simplify(m.view(), smac_path);},
    "unrepaired Smac corner fixture was unexpectedly safe");
  const auto safe = c::repair_grid_edges(m.view(), smac_path);
  const auto dense = c::resample(m.view(), c::simplify(m.view(), safe), .5);
  require(same(dense.front(), smac_path.front()) && same(dense.back(), smac_path.back()),
    "wall repair lost exact endpoints");
  for (std::size_t i = 1; i < dense.size(); ++i) {
    require(c::segment_clear(m.view(), dense[i - 1], dense[i]), "repaired wall detour is unsafe");
  }
}

void degenerate_and_invalid_inputs()
{
  Map m;
  auto single = c::simplify(m.view(), {{3.1, 4.2}, {3.1, 4.2}, {3.1, 4.2}});
  require(single.size() == 1, "duplicate single point not compacted");
  require(c::resample(m.view(), single, .5).size() == 1, "single point not preserved");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  require(!c::segment_clear(m.view(), {nan, 1.}, {2., 3.}), "nonfinite line accepted");
  auto invalid = m.view(); invalid.cells = nullptr;
  require(!c::segment_clear(invalid, {1., 1.}, {2., 2.}), "null grid accepted");
  rejects([&] {c::prepare_grid(invalid, .5);}, "invalid prepared grid accepted");
  rejects([&] {c::prepare_grid(m.view(), -1.);}, "negative footprint accepted");
  rejects([&] {c::prepare_grid(m.view(), nan);}, "nonfinite footprint accepted");
  rejects([&] {c::simplify(m.view(), {});}, "empty path accepted");
  rejects([&] {c::simplify(m.view(), {{1., 1.}, {nan, 2.}});}, "nonfinite path accepted");
  for (double spacing : {0., -1., 1.1, nan}) {
    rejects([&] {c::resample(m.view(), {{1., 1.}, {2., 2.}}, spacing);}, "invalid spacing accepted");
  }
  rejects([&] {c::resample(m.view(), {{1., 1.}, {2., 2.}}, 1e-20);}, "unbounded output allocation");
}

void budgets_interrupt_all_stages()
{
  Map m;
  auto cancel = [] {throw std::runtime_error("canceled");};
  rejects([&] {c::prepare_grid(m.view(), .5, cancel);}, "prepare skipped cancellation");
  rejects([&] {c::segment_clear(m.view(), {1., 1.}, {18., 18.}, cancel);}, "line skipped cancellation");
  rejects([&] {c::repair_grid_edges(m.view(), {{1., 1.}, {18., 18.}}, cancel);}, "repair skipped cancellation");
  rejects([&] {c::simplify(m.view(), {{1., 1.}, {18., 18.}}, cancel);}, "simplify skipped cancellation");
  rejects([&] {c::resample(m.view(), {{1., 1.}, {18., 18.}}, .5, cancel);}, "sample skipped cancellation");
  unsigned calls = 0;
  rejects([&] {c::prepare_grid(m.view(), .5, [&] {
      if (++calls == 3) {throw std::runtime_error("deadline");}
    });}, "prepare did not poll while working");
  calls = 0;
  rejects([&] {c::resample(m.view(), {{1.5, 1.5}, {18.5, 18.5}}, .001, [&] {
      if (++calls == 8) {throw std::runtime_error("deadline");}
    });}, "dense output did not poll while working");
}

// Independent closed-rectangle intersection reference (not a grid traversal).
bool intersects(c::Point a, c::Point b, double xmin, double ymin, double xmax, double ymax)
{
  double low = 0., high = 1.;
  for (const auto & axis : std::vector<std::vector<double>>{
      {a.x, b.x - a.x, xmin, xmax}, {a.y, b.y - a.y, ymin, ymax}})
  {
    if (std::abs(axis[1]) < 1e-14) {
      if (axis[0] < axis[2] - 1e-10 || axis[0] > axis[3] + 1e-10) {return false;}
    } else {
      double x = (axis[2] - axis[0]) / axis[1], y = (axis[3] - axis[0]) / axis[1];
      if (x > y) {std::swap(x, y);}
      low = std::max(low, x); high = std::min(high, y);
      if (low > high + 1e-10) {return false;}
    }
  }
  return true;
}
void randomized_supercover_matches_closed_cell_reference()
{
  Map m;
  std::mt19937 generator(1747);
  std::uniform_real_distribution<double> point(.1, 19.9);
  std::uniform_int_distribution<unsigned> cell(0, 19);
  for (unsigned trial = 0; trial < 1800; ++trial) {
    m.cells.assign(m.width * m.height, 0);
    for (unsigned j = 0; j < 12; ++j) {m.put(cell(generator), cell(generator), 253);}
    c::Point a{point(generator), point(generator)}, b{point(generator), point(generator)};
    if (trial % 4 == 0) {a.x = std::round(a.x); b.x = a.x;}
    if (trial % 7 == 0) {a.y = std::round(a.y); b.y = a.y;}
    bool expected = a.x > 0. && a.x < 20. && a.y > 0. && a.y < 20. &&
      b.x > 0. && b.x < 20. && b.y > 0. && b.y < 20.;
    for (unsigned y = 0; expected && y < 20; ++y) {
      for (unsigned x = 0; expected && x < 20; ++x) {
        if (m.cells[y * 20 + x] >= 253 && intersects(a, b, x, y, x + 1., y + 1.)) {
          expected = false;
        }
      }
    }
    require(c::segment_clear(m.view(), a, b) == expected,
      "supercover disagreed with independent closed-cell reference at trial " + std::to_string(trial));
    require(c::segment_clear(m.view(), b, a) == expected, "supercover was not reversible");
  }
}

int main()
{
  unsigned count = 0;
  for (const auto & test : std::vector<std::pair<const char *, std::function<void()>>>{
      {"free_lines_and_exact_endpoints", free_lines_and_exact_endpoints},
      {"blocked_unknown_and_boundary_contacts", blocked_unknown_and_boundary_contacts},
      {"fractional_world_and_subcell_collision", fractional_world_and_subcell_collision},
      {"unknown_and_exterior_footprint_clearance", unknown_and_exterior_footprint_clearance},
      {"greedy_detour_and_invalid_fallback", greedy_detour_and_invalid_fallback},
      {"local_smac_corner_repair", local_smac_corner_repair},
      {"degenerate_and_invalid_inputs", degenerate_and_invalid_inputs},
      {"budgets_interrupt_all_stages", budgets_interrupt_all_stages},
      {"randomized_supercover_matches_closed_cell_reference", randomized_supercover_matches_closed_cell_reference}})
  {
    try {test.second();} catch (const std::exception & error) {
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n'; return 1;
    }
    std::cout << "PASS " << test.first << '\n'; ++count;
  }
  std::cout << count << " C++ straight planner test groups passed\n";
}
