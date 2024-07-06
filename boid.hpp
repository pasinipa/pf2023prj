#pragma once
#include "simpars.hpp"
#include <array>
#include <iostream>
#include <variant>
#include <vector>

const float X_SPACE{1600.f};
const float Y_SPACE{900.f};
const float MAX_SPEED{1.5f};
const float MAX_IMPULSE{0.06f};

using ArrayF2 = std::array<float, 2>;

struct Obstacle
{
  Obstacle();
  ArrayF2 position;
};

class Boid
{
 public:
  Boid();
  Boid(ArrayF2 const& pos, ArrayF2 const& vel);
  void updatePosition();
  void updateVelocity();
  void updateImpulse(std::vector<Boid> const& flock,
                     std::vector<Obstacle> const& obstacles);
  ArrayF2 const& getPosition() const;
  ArrayF2 const& getVelocity() const;

 private:
  using Neighbour    = std::pair<Boid const&, float>;
  using NearObstacle = std::pair<Obstacle const&, float>;

  std::vector<Neighbour> m_neighbourhood;
  std::vector<NearObstacle> m_nearObstacles;
  ArrayF2 m_position;
  ArrayF2 m_velocity;
  ArrayF2 m_impulse;

  ArrayF2 impulseFromNeighbours();
  ArrayF2 impulseFromObstacles();
  void enforceToroidalSpace();
  void enforceBoundaries();
  void updateSurroundings(std::vector<Boid> const& state,
                          std::vector<Obstacle> const& obstacles);
};
