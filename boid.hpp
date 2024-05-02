#pragma once
#include "simpars.hpp"
#include <array>
#include <iostream>
#include <vector>

// const double SIGHT_DIST = 10;
using ArrayD2 = std::array<double, 2>;
constexpr double X_SPACE{ 100. };
constexpr double Y_SPACE{ 100. };
constexpr double MAX_COMPONENT_SPEED{ 1. };

class Boid {

    public:
    void update (std::vector<Boid> const& state, SimPars const& pars);
    ArrayD2 const& getPosition () const;
    Boid ();

    private:
    ArrayD2 position;
    ArrayD2 velocity;
    ArrayD2 impulse;

    // it should always be that size(nearBoids) == size(nearBoidsDistances)
    // should nearBoids be a vector pointer or just a vector?
    std::vector<const Boid*> nearBoids;
    std::vector<double> nearBoidsDist;

    ArrayD2 separationImpulse (SimPars const&) const;
    ArrayD2 allignmentImpulse (SimPars const&) const;
    ArrayD2 cohesionImpulse (SimPars const&) const;


    void updatePosition ();
    void updateVelocity ();
    void updateImpulse (SimPars const&);
    void updateNeighbours (std::vector<Boid> const& state, SimPars const& pars);
    // have to implement delta time
};
