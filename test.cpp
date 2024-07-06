#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <chrono>
#include <random>
#define private public

#include "boid.hpp"
#include "arrayop.hpp"
#include "simulation.hpp"
SimPars Simulation::parameters;
auto const& parameters{Simulation::parameters};


TEST_CASE("Testing array operations")
{
  ArrayF2 a1{1., 3.};
  ArrayF2 a2{-2., 0.};
  float k = 2.;

  auto sum  = a1 + a2;
  auto diff = a1 - a2;
  auto prod = a1 * k;
  auto quot = a2 / k; // a voler essere precisi bisognerebbe escludere che divida per 0...

  CHECK(sum == ArrayF2{-1., 3.});
  CHECK(diff == ArrayF2{3., 3.});
  CHECK(prod == ArrayF2{2., 6.});
  CHECK(quot == ArrayF2{-1, 0.});

  a1 += a2;
  a1 *= k;
  a2 /= k;
  CHECK(a1 == ArrayF2{-2., 6.});
  CHECK(a2 == ArrayF2{-1, .0});
}


Boid::Boid(ArrayF2 const& pos, ArrayF2 const& vel)
    : m_position{pos}
    , m_velocity{vel}
    , m_impulse{0.f, 0.f}
    , m_neighbourhood{}
    , m_nearObstacles{}
{}

Simulation::Simulation(std::vector<Boid> const& f)
    : flock{f}
    , obstacles{std::vector<Obstacle>()}
{}

std::vector<Boid> testFlock{Boid{{1., 2.}, {0., 1.}}, Boid{{4., 2.}, {0., 1.}},
                            Boid{{1., 6.}, {0., 1.}}};

Simulation sim{testFlock};

TEST_CASE("TESTING STATISTICS")
{
  FlightStatistics stat{sim.gatherData()};

  std::cout << stat.meanDist << '\n';
}
