#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <chrono>
#include <random>
#define private public

#include "arrayop.hpp"
#include "boid.hpp"
#include "simulation.hpp"

TEST_CASE("Testing array operations")
{
  ArrayF2 a1{1., 3.};
  ArrayF2 a2{-2., 0.};
  float k = 2.;

  auto sum  = a1 + a2;
  auto diff = a1 - a2;
  auto prod = a1 * k;
  auto quot =
      a2
      / k; // a voler essere precisi bisognerebbe escludere che divida per 0...

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

SimPars Simulation::parameters;

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

/*FlightStatistics Simulation::gatherData() const
{
  float meanDist{0.f};
  float meanSquaredDist{0.f};
  ArrayF2 meanVel{0.f, 0.f};
  ArrayF2 meanSquaredVel{0.f, 0.f};

  for (auto it = flock.begin(); it != flock.end(); ++it) {
    ArrayF2 vel{it->getVelocity()};
    meanVel += vel;
    meanSquaredVel += {std::pow(vel[0], 2.f), std::pow(vel[1], 2.f)};

    if (parameters.boidNumber < 2)
      continue;
    for (auto jt = it + 1; jt != flock.end(); ++jt) {
      float dist = euclidianNorm(it->getPosition() - jt->getPosition());
      meanDist += dist;
      meanSquaredDist += std::pow(dist, 2.f);
    }
  }
  // combinations without repetitions of flock.size() elements
  float nDist{
      static_cast<float>(parameters.boidNumber * (parameters.boidNumber - 1))};
  meanDist /= nDist;
  meanSquaredDist /= nDist;
  meanVel /= parameters.boidNumber;
  meanSquaredVel /= parameters.boidNumber;
  float varianceDist{meanSquaredDist - std::pow(meanDist, 2.f)};
  float stdDevDist{std::sqrt(varianceDist)};
  ArrayF2 varianceVel{
      meanSquaredVel
      - ArrayF2{std::pow(meanVel[0], 2.f), std::pow(meanVel[1], 2.f)}};
  ArrayF2 stdDevVel{std::sqrt(varianceVel[0]), std::sqrt(varianceVel[1])};
  return {meanDist, stdDevDist, meanVel, stdDevVel};
}*/

TEST_CASE("TESTING STATISTICS")
{
  FlightStatistics stat{sim.gatherData()};

  for (auto& b : sim.flock) {
    std::cout << stat.meanDist << '\n';
  }
}
