#include "simpars.hpp"
#include "simulation.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>

SimPars Simulation::parameters;
auto const& parameters{Simulation::parameters};

void handleCLInput(int argc, char* const argv[])
{
  bool isFetchingOpt{true};
  auto& p{Simulation::parameters};

  while (isFetchingOpt) {
    int c{getopt(argc, argv, "hbn:D:d:s:a:c:")};
    switch (c) {
    case 'n':
      p.boidNumber = std::atoi(optarg);
      break;
    case 'D':
      p.perceptionRadius = std::stof(std::string(optarg));
      break;
    case 'd':
      p.separationRadius = std::stof(std::string(optarg));
      break;
    case 's':
      p.separationStrength = std::stof(std::string(optarg));
      break;
    case 'a':
      p.allignmentStrength = std::stof(std::string(optarg));
      break;
    case 'c':
      p.cohesionStrength = std::stof(std::string(optarg));
      break;
    case 'b':
      p.boundariesEnabled = true;
      break; 
    case 'o':
      p.obstacleNumber = std::atoi(optarg);
      break;
    case 'r':
      p.sampleRate = std::atoi(optarg);
      break;
    case 'h':
    default:
      throw std::runtime_error {"Usage: [[insert usage here]]"};
      return;
    case EOF:
      isFetchingOpt = false;
      break;
    }
  }
  p.checkValidity();
}

int main(int argc, char* const argv[])
{
  try {
    handleCLInput(argc, argv);
  }
  catch (std::invalid_argument const& inputErr) {
      std::cout << "SYNTAX ERROR - Unexpected input format: " << inputErr.what() << std::endl;
      std::cout << "Try using -h flag for help." << std::endl;
      return 1;
  } catch (std::runtime_error const& e) {
      std::cout << e.what() << std::endl;
      return 1;
  }

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
