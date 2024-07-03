#pragma once
#include "simpars.hpp"
#include <array>
#include <iostream>
#include <vector>
#include "walls.hpp"

const double X_SPACE{800.};
const double Y_SPACE{600.};
const double MAX_SPEED{1.};

using ArrayD2 = std::array<float, 2>;

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

  ArrayD2 const& getPosition() const;

 private:
  using Neighbour = std::pair<Boid*, double>;

  std::vector<Neighbour> m_neighbourhood;
  ArrayD2 m_position;
  ArrayD2 m_velocity;
  ArrayD2 m_impulse;

  void enforceSpeedLimit();
  void updateNeighbourhood(std::vector<Boid> const& state);
  ArrayD2 separationImpulse();
  ArrayD2 allignmentImpulse();
  ArrayD2 cohesionImpulse();
};
