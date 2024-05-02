#pragma once
#include "boid.hpp"
#include "simpars.hpp"
#include <vector>

enum class Direction { FORWARDS = 1, BACKWARDS = -1 };

class Simulation {

    public:
    void updateSimulation ();

    Simulation (int);
    Simulation (int, SimPars const&);

    private:
    long long time{ 0 };
    std::vector<Boid> state{ std::vector<Boid> () };
    SimPars parameters{ SimPars () };

    void initState (int boidNum);
    void updateState ();
    void updateView ();
};
