#pragma once
#include "boid.hpp"

class Walls
{
 private:
  ArrayD2
      nearWall; // the area in which the boids "feel" the presence of the wall

  // function that checks if the boid hits the wall and modifies its velocity
  void boidImpact(Boid b, Walls const& w);
};