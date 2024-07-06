#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "simpars.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <fstream>
#include <vector>

struct FlightStatistics
{
  float meanDist{0.f};
  float stdDevDist{0.f};
  ArrayF2 meanVel{0.f, 0.f};
  ArrayF2 stdDevVel{0.f, 0.f};
};

class Simulation
{
 public:
  Simulation();
  Simulation(std::vector<Boid> const& f);
  long long time{0};
  void updateState();
  void updateView(sf::RenderWindow&) const;
  void streamStatsToFile(std::ofstream&) const;
  static SimPars parameters;

 private:
  std::vector<Boid> flock;
  std::vector<Obstacle> obstacles;

  FlightStatistics gatherData() const;
};
#endif
