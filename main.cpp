#include "simpars.hpp"
#include "simulation.hpp"
#include <unistd.h>
#include <iostream>
#include <string>

SimPars Simulation::parameters;

int main(int argc, char* const argv[])
{
  char c;
  // ES.40!
  while ((c = getopt(argc, argv, "hvn:D:d:s:a:c:")) != EOF) {
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
      return 1;
    }
  }

  Simulation sim;

  std::cin.get();
}
