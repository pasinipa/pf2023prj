#include "boid.hpp"
#include "arrayop.hpp"
#include <cmath>
#include <random>
#include <vector>

using ArrayD2 = std::array<double, 2>;

Boid::Boid () {
    position[0] = (std::rand () / static_cast<double> (RAND_MAX)) * X_SPACE;
    position[1] = (std::rand () / static_cast<double> (RAND_MAX)) * Y_SPACE;
    velocity[0] = ((std::rand () / static_cast<double> (RAND_MAX)) - 0.5) * MAX_COMPONENT_SPEED;
    velocity[1] = ((std::rand () / static_cast<double> (RAND_MAX)) - 0.5) * MAX_COMPONENT_SPEED;
    impulse = { 0, 0 };
}

ArrayD2 Boid::separationImpulse (SimPars const& pars) const {
    ArrayD2 sepImp = { 0, 0 };
    // assert (size(nearBoids) == size(nearBoidsDist));
    // range-for statement is not used because the index is needed to access
    // nearBoidsDist
    // also, need to choose a consistent naming criterion for the impulse
    // functions' scope variables
    for (int i = 0; i < static_cast<int> (size (nearBoids)); ++i) {
        if (nearBoidsDist[i] < pars.ds) {
            sepImp += (position - nearBoids[i]->position);
        }
    }
    sepImp *= pars.s;
    return sepImp;
}

ArrayD2 Boid::allignmentImpulse (SimPars const& pars) const {
    ArrayD2 sumVel{ 0, 0 };

    for (auto b : nearBoids) {
        sumVel += b->velocity;
    }
    ArrayD2 averageVel = sumVel / static_cast<double> (size (nearBoids));
    return averageVel * pars.a;
}


ArrayD2 Boid::cohesionImpulse (SimPars const& pars) const {
    ArrayD2 sumPos{ 0, 0 };

    for (auto b : nearBoids) {
        sumPos += b->position;
    }
    ArrayD2 averagePos = sumPos / static_cast<double> (size (nearBoids));
    return averagePos * pars.c;
}

void Boid::updatePosition () {
    position += velocity;

    // torus-like space
    while (position[0] > X_SPACE) {
        position[0] -= X_SPACE;
    }
    while (position[1] > Y_SPACE) {
        position[1] -= Y_SPACE;
    }
    while (position[0] < 0) {
        position[0] += X_SPACE;
    }
    while (position[1] < 0) {
        position[1] += Y_SPACE;
    }
}

void Boid::updateVelocity () {
    velocity += impulse;

    // enforcing the speed limit
    if (velocity[0] > MAX_COMPONENT_SPEED) {
        velocity[0] = MAX_COMPONENT_SPEED;
    }
    if (velocity[0] < -MAX_COMPONENT_SPEED) {
        velocity[0] = -MAX_COMPONENT_SPEED;
    }
    if (velocity[1] > MAX_COMPONENT_SPEED) {
        velocity[1] = MAX_COMPONENT_SPEED;
    }
    if (velocity[1] < -MAX_COMPONENT_SPEED) {
        velocity[1] = -MAX_COMPONENT_SPEED;
    }
}

void Boid::updateImpulse (SimPars const& pars) {

    impulse = this->separationImpulse (pars) + this->allignmentImpulse (pars) +
    this->cohesionImpulse (pars);
}

ArrayD2 const& Boid::getPosition () const {
    return position;
}

// the toroidal nature of the simulation space has to be taken into account!!
// both here and in the impulse functions...
double distance (Boid const& b1, Boid const& b2) {
    return std::sqrt (std::pow (b1.getPosition ()[0] - b2.getPosition ()[0], 2) +
    std::pow (b1.getPosition ()[1] - b2.getPosition ()[1], 2));
}

// this really could use some more readability
void Boid::updateNeighbours (std::vector<Boid> const& state, SimPars const& pars) {

    nearBoids.clear ();
    nearBoidsDist.clear ();

    for (auto& b : state) {
        double dist{ -1 };
        if (&b != this) {
            dist = distance (*this, b);
        }

        if (dist < pars.d and dist >= 0) {
            nearBoids.push_back (&b);
            nearBoidsDist.push_back (dist);
        }
    }
}

void Boid::update (std::vector<Boid> const& state, SimPars const& pars) {
    this->updateNeighbours (state, pars);
    this->updateImpulse (pars);
    this->updateVelocity ();
    this->updatePosition ();
}
