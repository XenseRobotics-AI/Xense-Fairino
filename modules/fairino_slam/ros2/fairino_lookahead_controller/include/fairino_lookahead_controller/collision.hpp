#pragma once

// ROS-independent geometry used by both the runtime collision check and tests.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "fairino_lookahead_controller/core.hpp"

namespace fairino_lookahead_controller::collision
{
using core::Point;
using core::Pose;
using core::Velocity;

struct Grid
{
  const unsigned char * cells;
  unsigned width, height;
  double resolution, origin_x, origin_y;
};

inline double cross(Point a, Point b, Point c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool on_segment(Point p, Point a, Point b)
{
  constexpr double epsilon = 1e-10;
  return std::abs(cross(a, b, p)) <= epsilon &&
         p.x >= std::min(a.x, b.x) - epsilon && p.x <= std::max(a.x, b.x) + epsilon &&
         p.y >= std::min(a.y, b.y) - epsilon && p.y <= std::max(a.y, b.y) + epsilon;
}

inline bool segments_intersect(Point a, Point b, Point c, Point d)
{
  const double ab_c = cross(a, b, c), ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a), cd_b = cross(c, d, b);
  if (((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0)) &&
    ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0)))
  {
    return true;
  }
  return on_segment(a, c, d) || on_segment(b, c, d) ||
         on_segment(c, a, b) || on_segment(d, a, b);
}

inline bool contains(const std::vector<Point> & polygon, Point p)
{
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto a = polygon[j], b = polygon[i];
    if (on_segment(p, a, b)) {return true;}
    if ((a.y > p.y) != (b.y > p.y) &&
      p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
    {
      inside = !inside;
    }
  }
  return inside;
}

inline bool overlaps_cell(const std::vector<Point> & polygon, double x, double y, double size)
{
  const std::vector<Point> corners{{x, y}, {x + size, y}, {x + size, y + size}, {x, y + size}};
  for (const auto p : polygon) {
    if (p.x >= x && p.x <= x + size && p.y >= y && p.y <= y + size) {return true;}
  }
  for (const auto c : corners) {if (contains(polygon, c)) {return true;}}
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    for (std::size_t j = 0; j < corners.size(); ++j) {
      if (segments_intersect(polygon[i], polygon[(i + 1) % polygon.size()],
        corners[j], corners[(j + 1) % corners.size()])) {return true;}
    }
  }
  return false;
}

inline bool footprint_clear(const Grid & grid, const std::vector<Point> & footprint, const Pose & pose)
{
  if (!grid.cells || !grid.width || !grid.height || !std::isfinite(grid.resolution) ||
    !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y) ||
    grid.resolution <= 0 || footprint.size() < 3 || !std::isfinite(pose.x) ||
    !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
  {
    return false;
  }
  std::vector<Point> polygon;
  polygon.reserve(footprint.size());
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  double min_x = INFINITY, min_y = INFINITY, max_x = -INFINITY, max_y = -INFINITY;
  for (const auto p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return false;}
    const Point transformed{pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
    polygon.push_back(transformed);
    min_x = std::min(min_x, transformed.x); max_x = std::max(max_x, transformed.x);
    min_y = std::min(min_y, transformed.y); max_y = std::max(max_y, transformed.y);
  }
  if (min_x < grid.origin_x || min_y < grid.origin_y ||
    max_x >= grid.origin_x + grid.width * grid.resolution ||
    max_y >= grid.origin_y + grid.height * grid.resolution) {return false;}
  const auto x0 = static_cast<unsigned>((min_x - grid.origin_x) / grid.resolution);
  const auto y0 = static_cast<unsigned>((min_y - grid.origin_y) / grid.resolution);
  const auto x1 = static_cast<unsigned>((max_x - grid.origin_x) / grid.resolution);
  const auto y1 = static_cast<unsigned>((max_y - grid.origin_y) / grid.resolution);
  for (unsigned y = y0; y <= y1; ++y) {
    for (unsigned x = x0; x <= x1; ++x) {
      // 254 is LETHAL_OBSTACLE, 255 is NO_INFORMATION. Check the polygon interior
      // as well as its edges so an obstacle enclosed by the robot is not missed.
      if (grid.cells[y * grid.width + x] >= 254 && overlaps_cell(polygon,
        grid.origin_x + x * grid.resolution, grid.origin_y + y * grid.resolution,
        grid.resolution)) {return false;}
    }
  }
  return true;
}

inline Pose integrate(const Pose & start, const Velocity & body, double time)
{
  double x = body.vx * time, y = body.vy * time;
  if (std::abs(body.wz) > 1e-9) {
    const double angle = body.wz * time;
    x = (body.vx * std::sin(angle) + body.vy * (std::cos(angle) - 1.)) / body.wz;
    y = (body.vx * (1. - std::cos(angle)) + body.vy * std::sin(angle)) / body.wz;
  }
  const double c = std::cos(start.yaw), s = std::sin(start.yaw);
  return {start.x + c * x - s * y, start.y + s * x + c * y, start.yaw + body.wz * time};
}

inline bool sweep_clear(
  const Grid & grid, const std::vector<Point> & footprint, const Pose & pose,
  const Velocity & command, double horizon, double maximum_step)
{
  if (!std::isfinite(horizon) || horizon < 0 || !std::isfinite(maximum_step) ||
    maximum_step <= 0 || !std::isfinite(command.vx) || !std::isfinite(command.vy) ||
    !std::isfinite(command.wz)) {return false;}
  double radius = 0.;
  for (const auto p : footprint) {radius = std::max(radius, std::hypot(p.x, p.y));}
  const double edge_speed = std::hypot(command.vx, command.vy) + radius * std::abs(command.wz);
  const double step = std::min(maximum_step, grid.resolution / (2. * std::max(edge_speed, 1e-9)));
  if (!std::isfinite(step) || step <= 0) {return false;}
  const double count = std::ceil(horizon / step);
  if (count > 4000) {return false;}  // Reject unbounded control-cycle work.
  const auto steps = static_cast<unsigned>(std::max(1., count));
  for (unsigned i = 0; i <= steps; ++i) {
    if (!footprint_clear(grid, footprint, integrate(pose, command, horizon * i / steps))) {
      return false;
    }
  }
  return true;
}
}  // namespace fairino_lookahead_controller::collision
