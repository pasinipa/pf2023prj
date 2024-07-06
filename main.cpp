#include "simpars.hpp"
#include "simulation.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

SimPars Simulation::parameters;
auto const& parameters{Simulation::parameters};

void handleCLInput(int argc, char* const argv[])
{
  bool isFetchingOpt{true};
  auto& p{Simulation::parameters};

  while (isFetchingOpt) {
    int c{getopt(argc, argv, "n:t:m:d:s:S:A:C:o:O:bh")};
    switch (c) {
    case 'n':
      p.boidNumber = std::atoi(optarg);
      break;
    case 't':
      p.sampleRate = std::atoi(optarg);
      break;
    case 'm':
      p.obstacleNumber = std::atoi(optarg);
      break;
    case 'd':
      p.perceptionRadius = std::stof(std::string(optarg));
      break;
    case 's':
      p.separationRadius = std::stof(std::string(optarg));
      break;
    case 'S':
      p.separationStrength = std::stof(std::string(optarg));
      break;
    case 'A':
      p.allignmentStrength = std::stof(std::string(optarg));
      break;
    case 'C':
      p.cohesionStrength = std::stof(std::string(optarg));
      break;
    case 'o':
      p.obstacleRadius = std::stof(std::string(optarg));
      break;
    case 'O':
      p.obstacleStrength = std::stof(std::string(optarg));
      break;
    case 'b':
      p.boundariesEnabled = true;
      break;
    case 'h':
    default:
      throw std::runtime_error{"USAGE ERROR!"};
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
  } catch (std::invalid_argument const& inputErr) {
    std::cout << "SYNTAX ERROR - Unexpected input format: " << inputErr.what()
              << std::endl;
    std::cout << "Try using -h flag for help." << std::endl;
    return 1;
  } catch (std::runtime_error const& e) {
    std::cout << e.what() << std::endl;
    std::cout << "List of options with argument type if required:" << std::endl;
    std::cout << "[flag] [arg_type] | description" << std::endl;
    std::cout << "  -n      int     | number of boids simulated" << std::endl;
    std::cout << "  -t      int     | sample rate (in time steps) of flight "
                 "data output"
              << std::endl;
    std::cout << "  -m      int     | number of obstacles simulated"
              << std::endl;
    std::cout << "  -d     float    | boid perception radius" << std::endl;
    std::cout << "  -s     float    | boid repulsion radius" << std::endl;
    std::cout << "  -S     float    | boid repulsion strength" << std::endl;
    std::cout << "  -A     float    | boid allignment strength" << std::endl;
    std::cout << "  -C     float    | boid cohesion strength" << std::endl;
    std::cout << "  -o     float    | obstacle influence radius" << std::endl;
    std::cout << "  -O     float    | obstacle repulsion strength" << std::endl;
    std::cout << "  -b              | activates simulation area boundaries (if "
                 "off, the space is torus-like)"
              << std::endl;
    return 1;
  }

  sf::RenderWindow window(sf::VideoMode(1600, 900), "Boid Simulator",
                          sf::Style::Close);
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
