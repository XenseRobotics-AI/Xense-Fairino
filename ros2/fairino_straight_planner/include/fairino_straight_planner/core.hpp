#ifndef FAIRINO_STRAIGHT_PLANNER__CORE_HPP_
#define FAIRINO_STRAIGHT_PLANNER__CORE_HPP_

// ROS-independent conservative checks on an already inflated global costmap.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fairino_straight_planner::core
{
struct Point {double x{0.}; double y{0.};};
struct Grid
{
  const unsigned char * cells{nullptr};
  unsigned width{0}, height{0};
  double resolution{0.}, origin_x{0.}, origin_y{0.};
  double clearance_radius{0.};
};
using Checkpoint = std::function<void()>;

struct PreparedGrid
{
  std::vector<unsigned char> cells;
  unsigned width{0}, height{0};
  double resolution{0.}, origin_x{0.}, origin_y{0.}, clearance_radius{0.};
  Grid view() const
  {
    return {cells.data(), width, height, resolution, origin_x, origin_y, clearance_radius};
  }
};

namespace detail
{
constexpr double epsilon = 1e-10;  // In grid coordinates, not world metres.
constexpr unsigned char blocked = 253;
constexpr unsigned char unknown = 255;

class Work
{
public:
  explicit Work(const Checkpoint & checkpoint) : checkpoint_(checkpoint)
  {
    if (checkpoint_) {checkpoint_();}
  }
  void step()
  {
    if (++steps_ == 64) {
      steps_ = 0;
      if (checkpoint_) {checkpoint_();}
    }
  }
private:
  const Checkpoint & checkpoint_;
  unsigned steps_{0};
};

inline bool finite(const Point & p) {return std::isfinite(p.x) && std::isfinite(p.y);}
inline bool same(const Point & a, const Point & b) {return a.x == b.x && a.y == b.y;}
inline bool valid(const Grid & g)
{
  return g.cells && g.width && g.height && std::isfinite(g.resolution) &&
         g.resolution > 0. && std::isfinite(g.origin_x) && std::isfinite(g.origin_y) &&
         std::isfinite(g.clearance_radius) && g.clearance_radius >= 0. &&
         static_cast<std::size_t>(g.width) <=
         std::numeric_limits<std::size_t>::max() / g.height &&
         std::isfinite(g.origin_x + g.width * g.resolution) &&
         std::isfinite(g.origin_y + g.height * g.resolution);
}
inline void require_grid(const Grid & g)
{
  if (!valid(g)) {throw std::invalid_argument("Invalid straight planner grid");}
}
inline Point normalized(const Grid & g, const Point & p)
{
  return {(p.x - g.origin_x) / g.resolution, (p.y - g.origin_y) / g.resolution};
}
inline bool inside(const Grid & g, const Point & p)
{
  const double margin = g.clearance_radius / g.resolution;
  // Contact with the unknown exterior is not a traversable segment.
  return finite(p) && p.x > margin + epsilon && p.y > margin + epsilon &&
         p.x < g.width - margin - epsilon && p.y < g.height - margin - epsilon;
}
inline bool cell_clear(const Grid & g, std::int64_t x, std::int64_t y, Work & work)
{
  work.step();
  return x >= 0 && y >= 0 && x < g.width && y < g.height &&
         g.cells[static_cast<std::size_t>(y) * g.width + static_cast<std::size_t>(x)] < blocked;
}
inline bool point_clear(const Grid & g, const Point & p, Work & work)
{
  if (!inside(g, p)) {return false;}
  const double rounded_x = std::round(p.x), rounded_y = std::round(p.y);
  const bool edge_x = std::abs(p.x - rounded_x) <= epsilon;
  const bool edge_y = std::abs(p.y - rounded_y) <= epsilon;
  const auto x = static_cast<std::int64_t>(edge_x ? rounded_x : std::floor(p.x));
  const auto y = static_cast<std::int64_t>(edge_y ? rounded_y : std::floor(p.y));
  // A point on a grid corner touches four closed cells; on an edge it touches two.
  for (int dx = edge_x ? -1 : 0; dx <= 0; ++dx) {
    for (int dy = edge_y ? -1 : 0; dy <= 0; ++dy) {
      if (!cell_clear(g, x + dx, y + dy, work)) {return false;}
    }
  }
  return true;
}
}  // namespace detail

// Existing 253/254 costs already encode the planning footprint. Only unknown
// space needs additional footprint clearance; inflating all costs again would
// double the robot radius. A candidate cell is blocked if ANY point in that
// closed cell can be within radius of an unknown cell. This is deliberately
// conservative at the raster boundary. The map exterior is handled by inside().
inline PreparedGrid prepare_grid(
  const Grid & raw, double clearance_radius, const Checkpoint & checkpoint = {})
{
  detail::require_grid(raw);
  if (!std::isfinite(clearance_radius) || clearance_radius < 0.) {
    throw std::invalid_argument("Invalid straight planner clearance radius");
  }
  detail::Work work(checkpoint);
  PreparedGrid out;
  out.width = raw.width; out.height = raw.height;
  out.resolution = raw.resolution; out.origin_x = raw.origin_x; out.origin_y = raw.origin_y;
  out.clearance_radius = clearance_radius;
  const std::size_t count = static_cast<std::size_t>(raw.width) * raw.height;
  out.cells.resize(count);
  std::vector<std::size_t> unknown_cells, free_cells;
  for (std::size_t i = 0; i < count; ++i) {
    work.step();
    out.cells[i] = raw.cells[i];
    if (raw.cells[i] == detail::unknown) {unknown_cells.push_back(i);}
    else if (raw.cells[i] < detail::blocked) {free_cells.push_back(i);}
  }
  if (clearance_radius == 0. || unknown_cells.empty() || free_cells.empty() ||
    2. * clearance_radius >= std::min(raw.width, raw.height) * raw.resolution)
  {
    return out;
  }
  const auto range_x = static_cast<std::int64_t>(std::min(
      static_cast<double>(raw.width - 1), std::ceil(clearance_radius / raw.resolution) + 1.));
  const auto range_y = static_cast<std::int64_t>(std::min(
      static_cast<double>(raw.height - 1), std::ceil(clearance_radius / raw.resolution) + 1.));
  std::vector<std::pair<std::int64_t, std::int64_t>> offsets;
  for (std::int64_t dy = -range_y; dy <= range_y; ++dy) {
    for (std::int64_t dx = -range_x; dx <= range_x; ++dx) {
      work.step();
      const double gap_x = static_cast<double>(std::max<std::int64_t>(std::abs(dx) - 1, 0));
      const double gap_y = static_cast<double>(std::max<std::int64_t>(std::abs(dy) - 1, 0));
      if (std::hypot(gap_x, gap_y) * raw.resolution <= clearance_radius +
        detail::epsilon * raw.resolution)
      {
        offsets.emplace_back(dx, dy);
      }
    }
  }
  // Scatter from unknowns or gather from traversable cells, whichever is smaller.
  const bool scatter = unknown_cells.size() <= free_cells.size();
  const auto & candidates = scatter ? unknown_cells : free_cells;
  for (const auto i : candidates) {
    const auto x = static_cast<std::int64_t>(i % raw.width);
    const auto y = static_cast<std::int64_t>(i / raw.width);
    for (const auto & offset : offsets) {
      work.step();
      const auto xx = x + offset.first, yy = y + offset.second;
      if (xx < 0 || yy < 0 || xx >= raw.width || yy >= raw.height) {continue;}
      const auto j = static_cast<std::size_t>(yy) * raw.width + static_cast<std::size_t>(xx);
      if (scatter) {
        if (raw.cells[j] < detail::blocked) {out.cells[j] = detail::unknown;}
      } else if (raw.cells[j] == detail::unknown) {
        out.cells[i] = detail::unknown;
        break;
      }
    }
  }
  return out;
}

// Check every closed cell touched by the complete line. Events are its crossings
// of grid lines; checking each event plus each interval interior implements a
// supercover, including lines along cell boundaries and corner-only contacts.
inline bool segment_clear(
  const Grid & grid, const Point & from, const Point & to, const Checkpoint & checkpoint = {})
{
  detail::Work work(checkpoint);
  if (!detail::valid(grid) || !detail::finite(from) || !detail::finite(to)) {return false;}
  const Point a = detail::normalized(grid, from), b = detail::normalized(grid, to);
  if (!detail::point_clear(grid, a, work) || !detail::point_clear(grid, b, work)) {return false;}
  const double dx = b.x - a.x, dy = b.y - a.y;
  if (dx == 0. && dy == 0.) {return true;}
  const int step_x = dx > 0. ? 1 : -1, step_y = dy > 0. ? 1 : -1;
  double boundary_x = dx > 0. ? std::floor(a.x) + 1. : std::floor(a.x);
  double boundary_y = dy > 0. ? std::floor(a.y) + 1. : std::floor(a.y);
  const double infinity = std::numeric_limits<double>::infinity();
  double tx = dx == 0. ? infinity : (boundary_x - a.x) / dx;
  double ty = dy == 0. ? infinity : (boundary_y - a.y) / dy;
  double t = 0.;
  const auto at = [&](double fraction) -> Point {
      return {a.x + dx * fraction, a.y + dy * fraction};
    };
  const std::size_t max_steps = static_cast<std::size_t>(grid.width) + grid.height + 4;
  for (std::size_t step = 0; step < max_steps; ++step) {
    const double next = std::min({1., tx, ty});
    if (!std::isfinite(next) || next < t - detail::epsilon) {return false;}
    if (next > t && !detail::point_clear(grid, at(t + (next - t) / 2.), work)) {return false;}
    if (!detail::point_clear(grid, at(next), work)) {return false;}
    if (next >= 1.) {return true;}
    if (tx <= next + detail::epsilon) {
      boundary_x += step_x;
      tx = dx == 0. ? infinity : (boundary_x - a.x) / dx;
    }
    if (ty <= next + detail::epsilon) {
      boundary_y += step_y;
      ty = dy == 0. ? infinity : (boundary_y - a.y) / dy;
    }
    t = next;
  }
  return false;
}


// Smac2D can join adjacent diagonal cell centres even when that edge touches a
// blocked side-cell corner. Replace only such local edges by a fully checked
// axis-aligned elbow. This is not a new search and never repairs blocked points,
// long jumps through walls, or two blocked alternatives.
inline std::vector<Point> repair_grid_edges(
  const Grid & grid, const std::vector<Point> & input, const Checkpoint & checkpoint = {})
{
  detail::require_grid(grid);
  detail::Work work(checkpoint);
  if (input.empty()) {throw std::invalid_argument("Cannot repair an empty path");}
  if (!segment_clear(grid, input.front(), input.front(), checkpoint)) {
    throw std::runtime_error("Fallback path starts in blocked space");
  }
  std::vector<Point> result{input.front()};
  for (std::size_t i = 1; i < input.size(); ++i) {
    work.step();
    const Point a = result.back(), b = input[i];
    if (!detail::finite(b)) {throw std::invalid_argument("Path contains non-finite point");}
    if (detail::same(a, b)) {continue;}
    if (segment_clear(grid, a, b, checkpoint)) {result.push_back(b); continue;}
    const Point ga = detail::normalized(grid, a), gb = detail::normalized(grid, b);
    const bool adjacent_diagonal =
      std::abs(std::floor(ga.x) - std::floor(gb.x)) == 1. &&
      std::abs(std::floor(ga.y) - std::floor(gb.y)) == 1.;
    bool repaired = false;
    if (adjacent_diagonal) {
      for (const Point elbow : {Point{a.x, b.y}, Point{b.x, a.y}}) {
        if (segment_clear(grid, a, elbow, checkpoint) &&
          segment_clear(grid, elbow, b, checkpoint))
        {
          result.push_back(elbow); result.push_back(b); repaired = true; break;
        }
      }
    }
    if (!repaired) {throw std::runtime_error("Fallback path has an unsafe unrepairable segment");}
  }
  return result;
}

inline std::vector<Point> simplify(
  const Grid & grid, const std::vector<Point> & input, const Checkpoint & checkpoint = {})
{
  detail::require_grid(grid);
  detail::Work work(checkpoint);
  if (input.empty()) {throw std::invalid_argument("Cannot simplify an empty path");}
  std::vector<Point> path;
  path.reserve(input.size());
  for (const auto & point : input) {
    work.step();
    if (!detail::finite(point)) {throw std::invalid_argument("Path contains non-finite point");}
    if (path.empty() || !detail::same(path.back(), point)) {path.push_back(point);}
  }
  if (!segment_clear(grid, path.front(), path.front(), checkpoint)) {
    throw std::runtime_error("Fallback path starts in blocked space");
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!segment_clear(grid, path[i - 1], path[i], checkpoint)) {
      throw std::runtime_error("Fallback path has an unsafe segment");
    }
  }
  std::vector<Point> result{path.front()};
  std::size_t current = 0;
  while (current + 1 < path.size()) {
    work.step();
    std::size_t farthest = path.size() - 1;
    while (farthest > current + 1 &&
      !segment_clear(grid, path[current], path[farthest], checkpoint))
    {
      --farthest;
    }
    result.push_back(path[farthest]);
    current = farthest;
  }
  return result;
}

inline std::vector<Point> resample(
  const Grid & grid, const std::vector<Point> & path, double spacing,
  const Checkpoint & checkpoint = {})
{
  detail::require_grid(grid);
  detail::Work work(checkpoint);
  if (path.empty() || !std::isfinite(spacing) || spacing <= 0. ||
    spacing > grid.resolution * (1. + detail::epsilon))
  {
    throw std::invalid_argument("Path spacing must be positive and no larger than map resolution");
  }
  if (!segment_clear(grid, path.front(), path.front(), checkpoint)) {
    throw std::runtime_error("Resampled path starts in blocked space");
  }
  constexpr std::size_t max_output_points = 1000000;  // Bound memory even for invalid external spacing.
  std::vector<Point> result{path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!segment_clear(grid, path[i - 1], path[i], checkpoint)) {
      throw std::runtime_error("Cannot resample an unsafe segment");
    }
    const double length = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (length == 0.) {continue;}
    const double steps_value = std::ceil(length / spacing);
    if (!std::isfinite(steps_value) || steps_value > max_output_points - result.size()) {
      throw std::runtime_error("Resampled path exceeds output point budget");
    }
    const auto steps = static_cast<std::size_t>(steps_value);
    for (std::size_t j = 1; j <= steps; ++j) {
      work.step();
      const double fraction = static_cast<double>(j) / static_cast<double>(steps);
      result.push_back(j == steps ? path[i] : Point{
          path[i - 1].x + fraction * (path[i].x - path[i - 1].x),
          path[i - 1].y + fraction * (path[i].y - path[i - 1].y)});
    }
  }
  return result;
}
}  // namespace fairino_straight_planner::core

#endif  // FAIRINO_STRAIGHT_PLANNER__CORE_HPP_
