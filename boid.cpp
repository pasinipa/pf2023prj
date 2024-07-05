#include "boid.hpp"
#include "arrayop.hpp"
#include "simulation.hpp"
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

extern SimPars const& parameters;

float euclidianNorm(ArrayF2 const& arr)
{
  float normSquared{
      std::inner_product(arr.begin(), arr.end(), arr.begin(), 0.f)};
  return std::sqrt(normSquared);
}

void setMagnitude(ArrayF2& arr, float magnitude)
{
  // what about division by zero?
  arr /= euclidianNorm(arr);
  arr *= magnitude;
}

void limitNorm(ArrayF2& arr, float normLimit)
{
  if (euclidianNorm(arr) > normLimit)
    setMagnitude(arr, normLimit);
}

void Boid::enforceToroidalSpace()
{
  if (m_position[0] > X_SPACE)
    m_position[0] -= X_SPACE;
  if (m_position[0] < 0)
    m_position[0] += X_SPACE;
  if (m_position[1] > Y_SPACE)
    m_position[1] -= Y_SPACE;
  if (m_position[1] < 0)
    m_position[1] += Y_SPACE;
}

static auto seed = static_cast<unsigned int>(
    std::chrono::steady_clock::now().time_since_epoch().count());
static std::default_random_engine eng(seed);
static std::uniform_real_distribution<float> xPosDistribution(0, X_SPACE);
static std::uniform_real_distribution<float> yPosDistribution(0, Y_SPACE);
static std::uniform_real_distribution<float> velDistribution(-MAX_SPEED,
                                                             MAX_SPEED);

Boid::Boid()
    : m_position{xPosDistribution(eng), yPosDistribution(eng)}
    , m_velocity{velDistribution(eng), velDistribution(eng)}
    , m_impulse{0.f, 0.f}
{
  // setMagnitude(m_velocity, MAX_SPEED);
}

ArrayF2 Boid::impulseFromNeighbours()
{
  ArrayF2 allignmentImp{0.f, 0.f};
  ArrayF2 separationImp{0.f, 0.f};
  ArrayF2 cohesionImp{0.f, 0.f};
  if (m_neighbourhood.empty())
    return {0.f, 0.f};

  for (auto& n : m_neighbourhood) {
    auto [otherBoid, distToOther] = n;
    cohesionImp += (otherBoid.m_position - m_position);
    allignmentImp += (otherBoid.m_velocity - m_velocity);
    if (distToOther != 0.f and distToOther < parameters.separationRuleRadius) {
      ArrayF2 repulsionFromOther{m_position - otherBoid.m_position};
      setMagnitude(repulsionFromOther, 1 / distToOther);
      separationImp += repulsionFromOther;
    }
  }

  float neighbourhoodSize{static_cast<float>(m_neighbourhood.size())};
  allignmentImp /= neighbourhoodSize;
  cohesionImp /= neighbourhoodSize;
  if (euclidianNorm(cohesionImp) < (parameters.separationRuleRadius + 5))
    cohesionImp = {0.f, 0.f};
  allignmentImp *= parameters.allignmentRuleStrength;
  cohesionImp *= parameters.cohesionRuleStrength;
  separationImp *= parameters.separationRuleStrength;

  return allignmentImp + cohesionImp + separationImp;
}

ArrayF2 Boid::impulseFromObstacles()
{
  // using std::accumulate??
  ArrayF2 obstacleImp{0.f, 0.f};
  if (m_nearObstacles.empty())
    return obstacleImp;

  for (auto const& o : m_nearObstacles) {
    auto [obstacle, distToObstacle] = o;
    if (distToObstacle != 0.f
        and distToObstacle < parameters.obstacleRuleRadius) {
      ArrayF2 repulsionFromObstacle{m_position - obstacle.position};
      setMagnitude(repulsionFromObstacle, 1 / distToObstacle);
      obstacleImp += repulsionFromObstacle;
    }
    obstacleImp *= parameters.obstacleRuleStrength;
  }
  return obstacleImp;
}

void Boid::updateImpulse(std::vector<Boid> const& flock,
                         std::vector<Obstacle> const& obstacles)
{
  updateSurroundings(flock, obstacles);
  m_impulse = impulseFromNeighbours() + impulseFromObstacles();
  limitNorm(m_impulse, MAX_IMPULSE);
}

/*
void Boid::wallDeviation(std::vector<Wall> const& wallsConfig)
{
  for (auto w : wallsConfig) {
    double dist = euclidianNorm(m_position - w.w_position);
    if (dist < w.radius) {
      ArrayD2 wallDev = (m_position - w.w_position) * 1/(dist *
euclidianNorm(m_position)) * Simulation::parameters.w; m_impulse += wallDev;
    }
  }
}

void Boid::edgeBounce() {
  if (((m_position + m_velocity)[0] >= X_SPACE) || ((m_position +
m_velocity)[0] <= 0)) m_velocity[0] *= -1; if (((m_position + m_velocity)[1]
>= Y_SPACE) || ((m_position + m_velocity)[1] <= 0)) m_velocity[1] *= -1;
}
 */

void Boid::updatePosition()
{
  m_position += m_velocity;
  enforceToroidalSpace();
}

void Boid::updateVelocity()
{
  m_velocity += m_impulse;
  setMagnitude(m_velocity, MAX_SPEED);
}

void Boid::updateSurroundings(std::vector<Boid> const& flock,
                              std::vector<Obstacle> const& obstacles)
{
  m_neighbourhood.clear();
  for (auto const& b : flock) {
    if (&b == this)
      continue;
    float distance{euclidianNorm(b.m_position - m_position)};
    if (distance < parameters.perceptionRadius)
      m_neighbourhood.push_back({b, distance});
  }

  m_nearObstacles.clear();
  for (auto const& o : obstacles) {
    float distance{euclidianNorm(o.position - m_position)};
    if (distance < parameters.perceptionRadius)
      m_nearObstacles.push_back({o, distance});
  }
}

ArrayF2 const& Boid::getPosition() const
{
  return m_position;
}
