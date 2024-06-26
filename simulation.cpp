#include "simulation.hpp"
#include <vector>

Simulation::Simulation()
    : state{std::vector<Boid>(Simulation::parameters.bNum)}
{}

void Simulation::updateState()
{
  for (auto& b : state) {
    b.updateImpulse(state);
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
