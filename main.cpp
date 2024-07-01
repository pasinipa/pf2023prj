#include "simpars.hpp"
#include "simulation.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <unistd.h>
#include <iostream>
#include <string>

SimPars Simulation::parameters;

// enum class UserMode { SpawnBoids = 0, SpawnObstacles, ClearBoids, ClearWalls
// };

bool handleCLInput(int argc, char* const argv[])
{
  bool isFetchingOpt{true};

  while (isFetchingOpt) {
    int c{getopt(argc, argv, "hvn:D:d:s:a:c:")};

    switch (c) {
    case 'v':
      Simulation::parameters.isVerbose = true;
      break;
    case 'n':
      Simulation::parameters.bNum = std::atoi(optarg);
      break;
    case 'D':
      Simulation::parameters.d = std::stod(std::string(optarg));
      break;
    case 'd':
      Simulation::parameters.ds = std::stod(std::string(optarg));
      break;
    case 's':
      Simulation::parameters.s = std::stod(std::string(optarg));
      break;
    case 'a':
      Simulation::parameters.a = std::stod(std::string(optarg));
      break;
    case 'c':
      Simulation::parameters.c = std::stod(std::string(optarg));
      break;
    case 'h':
    default:
      std::cout << "usage: you used it wrong, dumbass";
      return 0;
    case EOF:
      isFetchingOpt = false;
      break;
    }
  }
  return 1;
}

int main(int argc, char* const argv[])
{
  if (!handleCLInput(argc, argv))
    return 1;
  Simulation sim;

  sf::RenderWindow window;
  window.create(sf::VideoMode(800, 600), "My window", sf::Style::Close);
  window.setVerticalSyncEnabled(true);

  sf::Font robotoRegular;
  robotoRegular.loadFromFile("Roboto/Roboto-Regular.ttf");
  sf::Text text;
  text.setFont(robotoRegular);
  text.setCharacterSize(24);
  text.setFillColor(sf::Color::White);
  text.setString("Porcozio diopera");
  text.setPosition(300, 300);

  while (window.isOpen()) {
    sim.updateState();

    sf::Event event;
    while (window.pollEvent(event)) {
      switch (event.type) {
      case sf::Event::Closed:
        window.close();
        break;
      case sf::Event::MouseButtonPressed:
        // other way of doing this?
        break;
      default:
        break;
      }
    }

    window.clear();
    window.draw(text);
    sim.updateView(window);
    window.display();
  }
}
