#include "boid.hpp"
#include "arrayop.hpp"
#include "simulation.hpp"
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

float euclidianNorm(ArrayF2 const& arr)
{
  float normSquared{static_cast<float>(
      std::inner_product(arr.begin(), arr.end(), arr.begin(), 0.))};
  return std::sqrt(normSquared);
}

void Boid::enforceSpeedLimit()
{
  float norm{euclidianNorm(m_velocity)};
  if (norm > MAX_SPEED) {
    m_velocity /= norm;
    m_velocity *= MAX_SPEED;
  }
}

void Boid::enforceToroidalSpace()
{}

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
  enforceSpeedLimit();
}

void Boid::updateImpulse(std::vector<Boid> const& state)
{
  updateNeighbourhood(state);
  if (m_neighbourhood.empty()) {
    m_impulse = {0.f, 0.f};
    return;
  }

  ArrayF2 separationImp{0.f, 0.f};
  ArrayF2 cohesionImp{0.f, 0.f};
  ArrayF2 allignmentImp{0.f, 0.f};

  for (auto& n : m_neighbourhood) {
    auto [boidPtr, dist] = n;
    cohesionImp += (boidPtr->m_position - m_position);
    allignmentImp += (boidPtr->m_velocity - m_velocity);
  }

  allignmentImp /= static_cast<float>(m_neighbourhood.size());
  allignmentImp *= Simulation::parameters.a;
  cohesionImp /= static_cast<float>(m_neighbourhood.size());
  cohesionImp /= Simulation::parameters.d;
  cohesionImp *= MAX_SPEED;
  cohesionImp *= Simulation::parameters.c;

  std::cout << "Separation Impulse: " << separationImp[0] << ", "
            << separationImp[1] << std::endl;
  std::cout << "Cohesion Impulse after: " << cohesionImp[0] << ", "
            << cohesionImp[1] << std::endl;
  std::cout << "Allignment Impulse: " << allignmentImp[0] << ", "
            << allignmentImp[1] << std::endl;
  std::cout << "N-hood size: " << m_neighbourhood.size() << std::endl;

  m_impulse = separationImp + cohesionImp + allignmentImp;
  return;
}

void Boid::wallDeviation(std::vector<Wall> const& wallsConfig)
{
  for (auto w : wallsConfig) {
    double dist = euclidianNorm(m_position - w.w_position);
    if (dist < w.radius) {
      ArrayD2 wallDev = (m_position - w.w_position) * 1/(dist * euclidianNorm(m_position)) * Simulation::parameters.w;
      m_impulse += wallDev;
    }
  }
}

void Boid::edgeBounce() {
  if (((m_position + m_velocity)[0] >= X_SPACE) || ((m_position + m_velocity)[0] <= 0)) m_velocity[0] *= -1;
  if (((m_position + m_velocity)[1] >= Y_SPACE) || ((m_position + m_velocity)[1] <= 0)) m_velocity[1] *= -1;
}

void Boid::updatePosition()
{
  m_position += m_velocity;
  enforceToroidalSpace();
}

void Boid::updateVelocity()
{
  m_velocity += m_impulse;
  enforceSpeedLimit();
}

void Boid::updateNeighbourhood(std::vector<Boid> const& state)
{
  m_neighbourhood.clear();

  for (auto const& b : state) {
    if (&b == this)
      continue;

    float distance = euclidianNorm(b.m_position - this->m_position);
    if (distance < Simulation::parameters.d)
      m_neighbourhood.push_back(Neighbour{&b, distance});
  }
}

ArrayF2 const& Boid::getPosition() const
{
  return m_position;
}
