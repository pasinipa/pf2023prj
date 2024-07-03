#pragma once
#include "simpars.hpp"
#include <array>
#include <iostream>
#include <vector>
#include "walls.hpp"

const float X_SPACE{800.};
const float Y_SPACE{600.};
const float MAX_SPEED{1.};

using ArrayF2 = std::array<float, 2>;

class Boid
{
  friend class Statistics;
 public:
  Boid();
  Boid(ArrayD2 position, ArrayD2 velocity);

  void updateImpulse(std::vector<Boid> const& state);
  void updatePosition();
  void updateVelocity();
  void wallDeviation(std::vector<Wall> const& wallsConfig);
  void edgeBounce();
  ArrayF2 const& getPosition() const;

 private:
  using Neighbour = std::pair<Boid const*, float>;
  std::vector<Neighbour> m_neighbourhood;
  ArrayF2 m_position;
  ArrayF2 m_velocity;
  ArrayF2 m_impulse;

  void enforceToroidalSpace();
  void enforceSpeedLimit();
  void updateNeighbourhood(std::vector<Boid> const& state);
};
