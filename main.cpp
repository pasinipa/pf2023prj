#include "simpars.hpp"
#include "simulation.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>

SimPars Simulation::parameters;
auto const& parameters{Simulation::parameters};

void handleCLInput(int argc, char* const argv[])
{
  bool isFetchingOpt{true};
  auto& p{Simulation::parameters};

  while (isFetchingOpt) {
    int c{getopt(argc, argv, "hvn:D:d:s:a:c:")};

    switch (c) {
    case 'v':
      p.isVerbose = true;
      break;
    case 'n':
      p.boidNumber = std::atoi(optarg);
      break;
    case 'D':
      p.perceptionRadius = std::stof(std::string(optarg));
      break;
    case 'd':
      p.separationRuleRadius = std::stof(std::string(optarg));
      break;
    case 's':
      p.separationRuleStrength = std::stof(std::string(optarg));
      break;
    case 'a':
      p.allignmentRuleStrength = std::stof(std::string(optarg));
      break;
    case 'c':
      p.cohesionRuleStrength = std::stof(std::string(optarg));
      break;
    case 'h':
    default:
      std::cout << "usage: you used it wrong, dumbass";
      return;
    case EOF:
      isFetchingOpt = false;
      break;
      case 'e':
      Simulation::parameters.edges = true;
      break; 
    }
  }
}

int main(int argc, char* const argv[])
{
  // use a try/catch on this guy
  try {
    handleCLInput(argc, argv);
  }
  catch (std::runtime_error const& e) {}

  sf::RenderWindow window(sf::VideoMode(1600, 900), "Boid Simulator", sf::Style::Close);
  window.setVerticalSyncEnabled(true);
  std::ofstream outputFile{"output.txt"};
  outputFile
      << "|  time  |     distance      |             velocity              |"
      << std::endl;
  outputFile
      << "|        | average | std dev |     average     |     std dev     |"
      << std::endl;
  outputFile 
      << "|        |         |         |    x   |   y    |    x   |   y    |"
      << std::endl;

  Simulation sim;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
        break;
      }
    }
    sim.updateState();
    sim.streamStatsToFile(outputFile);
    window.clear();
    sim.updateView(window);
    window.display();
  }

  outputFile.close();
}
