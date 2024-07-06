#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <chrono>
#include <random>

#include "arrayop.hpp"
#include "boid.hpp"
#include "simulation.hpp"

SimPars Simulation::parameters;
auto const& parameters{Simulation::parameters};

TEST_CASE("Testing array operations")
{
  ArrayF2 a1{1.f, 3.f};
  ArrayF2 a2{-2.f, 0.f};
  float k{2.f};

  auto sum{a1 + a2};
  auto diff{a1 - a2};
  auto prod{a1 * k};
  auto quot{a2 / k};

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

TEST_CASE("TESTING STATISTICS")
{
  SUBCASE("testing with 3 boids")
  {
    std::vector<Boid> testFlock{Boid{{1., 2.}, {0., 1.}},
                                Boid{{4., 2.}, {0., 1.}},
                                Boid{{1., 6.}, {0., 1.}}};

    FlightStatistics stat{gatherData(testFlock)};

    CHECK(stat.meanDist == 4);
    CHECK(stat.stdDevDist == doctest::Approx(0.816).epsilon(0.001));

    CHECK(stat.meanVel[0] == 0.);
    CHECK(stat.meanVel[1] == 1.);
    CHECK(stat.stdDevVel[0] == 0.);
    CHECK(stat.stdDevVel[1] == 0.);
  }

  SUBCASE("testing with negative positions")
  {
    std::vector<Boid> testFlock{Boid{{-1., 2.}, {0., 1.}},
                                Boid{{4., -2.}, {0., 1.}},
                                Boid{{1., -6.}, {0., 1.}}};

    FlightStatistics stat{gatherData(testFlock)};

    CHECK(stat.meanDist == doctest::Approx(6.550).epsilon(0.001));
    CHECK(stat.stdDevDist == doctest::Approx(1.329).epsilon(0.001));

    CHECK(stat.meanVel[0] == 0.);
    CHECK(stat.meanVel[1] == 1.);
    CHECK(stat.stdDevVel[0] == 0.);
    CHECK(stat.stdDevVel[1] == 0.);
  }

  SUBCASE("testing with negative velocities")
  {
    std::vector<Boid> testFlock{Boid{{1., 2.}, {0., -1.}},
                                Boid{{4., 2.}, {1., -4.}},
                                Boid{{1., 6.}, {0., 1.}}};

    FlightStatistics stat{gatherData(testFlock)};

    CHECK(stat.meanDist == 4);
    CHECK(stat.stdDevDist == doctest::Approx(0.816).epsilon(0.001));

    CHECK(stat.meanVel[0] == doctest::Approx(0.333).epsilon(0.001));
    CHECK(stat.meanVel[1] == doctest::Approx(-1.333).epsilon(0.001));
    CHECK(stat.stdDevVel[0] == doctest::Approx(0.471).epsilon(0.001));
    CHECK(stat.stdDevVel[1] == doctest::Approx(2.055).epsilon(0.001));
  }
}
