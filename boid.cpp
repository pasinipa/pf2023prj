#include "boid.hpp"
#include "arrayop.hpp"
#include "simulation.hpp"
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

double euclidianNorm(ArrayD2 const& arr)
{
  double norm{0};
  norm = std::inner_product(arr.begin(), arr.end(), arr.begin(), 0.);
  norm = std::sqrt(norm);
  return norm;
}

void Boid::enforceSpeedLimit()
{
  double norm{euclidianNorm(m_velocity)};
  if (norm > MAX_SPEED) {
    m_velocity /= norm;
    m_velocity *= MAX_SPEED;
  }
}

std::default_random_engine eng;

Boid::Boid()
{
  std::uniform_real_distribution<float> xPosDistribution(0, X_SPACE);
  std::uniform_real_distribution<float> yPosDistribution(0, Y_SPACE);
  std::uniform_real_distribution<float> xVelDistribution(0, MAX_SPEED);
  std::uniform_real_distribution<float> yVelDistribution(0, MAX_SPEED);

  m_position = {xPosDistribution(eng), yPosDistribution(eng)};
  m_velocity = {xVelDistribution(eng), yVelDistribution(eng)};
  enforceSpeedLimit();
}

void Boid::updateImpulse(std::vector<Boid> const& state)
{
  updateNeighbourhood(state);

  ArrayD2 separationImp;
  ArrayD2 cohesionImp;
  ArrayD2 allignmentImp;

  for (auto& n : m_neighbourhood) {
    auto [boidPtr, dist] = n;

    if (dist < Simulation::parameters.ds)
      separationImp += (m_position - boidPtr->m_position);

    cohesionImp += boidPtr->m_position;
    allignmentImp += boidPtr->m_velocity;
  }
  separationImp *= Simulation::parameters.s;
  allignmentImp /= static_cast<double>(m_neighbourhood.size());
  allignmentImp *= Simulation::parameters.a;
  cohesionImp /= static_cast<double>(m_neighbourhood.size());
  cohesionImp *= Simulation::parameters.c;

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
  // torus-like space ??
}

void Boid::updateVelocity()
{
  m_velocity += m_impulse;

  // enforcing the speed limit
  double norm{euclidianNorm(m_velocity)};
  if (norm > MAX_SPEED) {
    m_velocity /= norm;
    m_velocity *= MAX_SPEED;
  }
}

void Boid::updateNeighbourhood(std::vector<Boid> const& state)
{
  m_neighbourhood.clear();

  for (auto b : state) {
    if (&b == this) {
      continue;
    }

    double dist = euclidianNorm(b.m_position - m_position);
    if (dist < Simulation::parameters.d) {
      m_neighbourhood.push_back(Neighbour{&b, dist});
    }
  }
}

ArrayD2 const& Boid::getPosition() const
{
  return m_position;
}
