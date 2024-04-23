#pragma once
#include "boid.hpp"
#include <vector>

enum Direction { FORWARDS, BACKWARDS };

class Simulation {

    long long time;
    std::vector<Boid>* state;
    SimulationParameters parameters;

    Simulation (int);
    Simulation (int, const SimulationParameters&);
    void initState (int boidNum);

    void updateState (SimulationParameters);
    void updateView ();

    void updateSimulation (SimulationParameters);
};
