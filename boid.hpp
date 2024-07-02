#pragma once
#include "simpars.hpp"
#include <array>
#include <iostream>
#include <vector>
#include "walls.hpp"

const double X_SPACE{100.};
const double Y_SPACE{100.};
const double MAX_SPEED{1.};

using ArrayD2 = std::array<double, 2>;

class Boid
{
  friend class Statistics;
 public:
  Boid(ArrayD2 position, ArrayD2 velocity);

  void updateImpulse(std::vector<Boid> const& state);
  void updatePosition();
  void updateVelocity();
  void wallDeviation(std::vector<Wall> const& wallsConfig);
  void edgeBounce();

 private:
  using Neighbour = std::pair<Boid*, double>;

  std::vector<Neighbour> m_neighbourhood;
  ArrayD2 m_position;
  ArrayD2 m_velocity;
  ArrayD2 m_impulse;

  void updateNeighbourhood(std::vector<Boid> const& state);
  ArrayD2 separationImpulse();
  ArrayD2 allignmentImpulse();
  ArrayD2 cohesionImpulse();
};
