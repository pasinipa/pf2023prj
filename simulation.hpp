#pragma once
#include "boid.hpp"
#include "simpars.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>

class Simulation
{
 public:
  void updateState();
  void updateView(sf::RenderWindow&);
  Simulation();

  static SimPars parameters;

  // !! REMEMBER TO PUT THIS PRIVATE AGAIN !!
  std::vector<Boid> state;
  // std::vector<Wall> wallsConfig;

 private:
  long long time{0};
};
