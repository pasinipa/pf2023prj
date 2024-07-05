#pragma once
#include "boid.hpp"
#include "simpars.hpp"
#include "statistics.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>

class Simulation
{
 public:
  Simulation();
  void updateState();
  void updateView(sf::RenderWindow&);
  static SimPars parameters;

 private:
  std::vector<Boid> flock;
  std::vector<Obstacle> obstacles;
  long long time{0};
};
