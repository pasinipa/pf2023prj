#include "simulation.hpp"
// #include "graphics.hpp"
#include <random>
#include <vector>


Simulation::Simulation (int boidNum) {
    initState (boidNum);
    parameters = SimulationParameters();
    time       = 0;
}

Simulation::Simulation (int boidNum, const SimulationParameters& pars) {
    initState (boidNum);
    parameters = pars;
    time       = 0;
}

void Simulation::initState (int boidNum) {
    state = new std::vector<Boid> (boidNum);
    for (auto b : *state) {
        b.x    = (std::rand () / (double)RAND_MAX) * X_SPACE;
        b.y    = (std::rand () / (double)RAND_MAX) * Y_SPACE;
        b.velX = (std::rand () / (double)RAND_MAX - 0.5) * MAX_COMPONENT_SPEED;
        b.velY = (std::rand () / (double)RAND_MAX - 0.5) * MAX_COMPONENT_SPEED;
        b.impX = 0;
        b.impY = 0;
    }
}

void Simulation::updateState (SimulationParameters pars) {
    for (auto b : *state) {
        b.updateNeighbours (*state, pars);
        b.updateImpulse (pars);
    }

    for (auto b : *state) {
        b.updateVelocity ();
        b.updatePosition ();
    }
}
// void updateView (Direction d);

void Simulation::updateSimulation (SimulationParameters pars) {
    updateState (pars);
    // updateView();

    ++time;
}
