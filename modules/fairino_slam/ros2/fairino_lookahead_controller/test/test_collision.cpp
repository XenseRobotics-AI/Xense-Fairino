#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "fairino_lookahead_controller/collision.hpp"

using namespace fairino_lookahead_controller;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main()
{
  std::vector<unsigned char> cells(100 * 100, 0);
  const collision::Grid grid{cells.data(), 100, 100, .02, -1., -1.};
  const std::vector<core::Point> footprint{{-.3, -.2}, {.3, -.2}, {.3, .2}, {-.3, .2}};
  auto obstacle = [&](double x, double y, unsigned char value) {
      cells[static_cast<unsigned>((y + 1.) / .02) * 100 +
        static_cast<unsigned>((x + 1.) / .02)] = value;
    };
  require(collision::footprint_clear(grid, footprint, {0, 0, 0}), "clear footprint rejected");
  obstacle(.02, .02, 254);
  require(!collision::footprint_clear(grid, footprint, {0, 0, 0}), "interior obstacle missed");
  obstacle(.02, .02, 255);
  require(!collision::footprint_clear(grid, footprint, {0, 0, 0}), "unknown cell accepted");
  obstacle(.02, .02, 253);
  require(collision::footprint_clear(grid, footprint, {0, 0, 0}), "inflation cost treated as lethal occupancy");
  obstacle(.02, .02, 0);
  require(!collision::footprint_clear(grid, footprint, {.8, 0, 0}), "map boundary escape accepted");

  obstacle(.35, .01, 254);
  require(collision::footprint_clear(grid, footprint, {0, 0, 0}), "forward obstacle already overlaps footprint");
  require(!collision::sweep_clear(grid, footprint, {0, 0, 0}, {.1, 0, 0}, 1., .05),
    "forward swept collision missed");
  require(collision::sweep_clear(grid, footprint, {0, 0, 0}, {-.1, 0, 0}, 1., .05),
    "reverse clear sweep rejected");
  obstacle(.35, .01, 0);
  obstacle(.01, .25, 254);
  require(!collision::sweep_clear(grid, footprint, {0, 0, 0}, {0, .1, 0}, 1., .05),
    "holonomic lateral collision missed");
  obstacle(.01, .25, 0);
  obstacle(.01, .31, 254);
  require(collision::footprint_clear(grid, footprint, {0, 0, 0}), "rotation obstacle initially occupied");
  require(!collision::sweep_clear(grid, footprint, {0, 0, 0}, {0, 0, 1.}, 1.5, .05),
    "rotating footprint collision missed");
  obstacle(.01, .31, 0);
  const auto integrated = collision::integrate({1., 2., 1.5707963267948966}, {.1, 0, 0}, 2.);
  require(std::abs(integrated.x - 1.) < 1e-10 && std::abs(integrated.y - 2.2) < 1e-10,
    "body velocity was integrated in wrong frame");
  require(!collision::sweep_clear(grid, footprint, {0, 0, 0}, {NAN, 0, 0}, 1., .05),
    "non-finite command accepted");
  require(!collision::footprint_clear(grid, {}, {0, 0, 0}), "empty footprint accepted");
  std::cout << "Collision tests passed: interior, unknown, boundary, XY sweep, rotation, frames, invalid input.\n";
}
