#include "simulation.hpp"
// #include "graphics.hpp"
#include <iostream>
#include <random>
#include <vector>


Simulation::Simulation (int boidNum) : state{ std::vector<Boid> (boidNum) } {
    std::cout << size (state) << std::endl;
    std::cout << state[0].getPosition ()[0] << " ---- "
              << state[1].getPosition ()[1] << std::endl;
    std::cout << parameters.d << std::endl;
    std::cin.get ();
}

Simulation::Simulation (int boidNum, SimPars const& pars)
: state{ std::vector<Boid> (boidNum) }, parameters{ pars } {
}


void Simulation::updateState () {
    for (auto b : state) {
        b.update (state, parameters);
    }
}

// void updateView ();
void Simulation::updateSimulation () {
    updateState ();
    // updateView();

    ++time;
}
