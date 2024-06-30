#include "simulation.hpp"
#include <vector>

Simulation::Simulation()
    : state{std::vector<Boid>(Simulation::parameters.bNum)}
{}

void Simulation::updateState()
{
  for (auto& b : state) {
    b.updateImpulse(state);
    //some way to determine the closest (if not the only) wall, say w1
    b.wallDeviation(wallsConfig);
  }
  for (auto& b : state) {
    b.updatePosition();
    b.updateVelocity();
  }
}

// void updateView ();

void Simulation::updateSimulation()
{
  updateState();
  // updateView();

  ++time;
}
