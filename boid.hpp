#pragma once
#include <array>
#include <vector>

// const double SIGHT_DIST = 10;

struct SimulationParameters {
    double ds, d, s, a, c;
    SimulationParameters (double par1, double par2, double par3, double par4, double par5) {
        ds = par1;
        d  = par2;
        s  = par3;
        a  = par4;
        c  = par5;
    }
    SimulationParameters () {
        *this = {1, 20, 5, 0.7, 0.3};
    }
};

const SimulationParameters STANDARD_SIM_PARAMETERS (1, 20, 5, 0.7, 0.3);
const double X_SPACE = 100., Y_SPACE = 100.;
const double MAX_COMPONENT_SPEED = 1.;

class Boid {

    double x, y;
    double velX, velY;
    double impX, impY;

    // it should always be that size(nearBoids) == size(nearBoidsDistances)
    std::vector<const Boid&> nearBoids;
    std::vector<double> nearBoidsDist;

    std::array<double, 2> separationImpulse (SimulationParameters pars);
    std::array<double, 2> allignmentImpulse (SimulationParameters pars);
    std::array<double, 2> cohesionImpulse (SimulationParameters pars);

    double distance (const Boid&);

    public:
    void updatePosition ();
    void updateVelocity ();
    void updateImpulse (SimulationParameters);
    void updateNeighbours (const std::vector<Boid>& state, SimulationParameters pars);

    // have to implement delta time
};
