#pragma once
#include "boid.hpp"
#include "simpars.hpp"
#include <vector>

class Simulation
{
 public:
  void updateSimulation();
  Simulation();

  static SimPars parameters;

 private:
  long long time{0};
  std::vector<Boid> state;
  std::vector<Wall> wallsConfig;

  void updateState();
};
