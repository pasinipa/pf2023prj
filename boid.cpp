#include "boid.hpp"
#include <vector>

// const double SIGHT_DIST = 10;

std::array<double, 2> Boid::separationImpulse (SimulationParameters pars) {

    std::array<double, 2> sepImp = { 0, 0 };

    // assert (size(nearBoids) == size(nearBoidsDist));
    for (int i = 0; i < (int)size (nearBoidsDist); ++i) {
        if (nearBoidsDist[i] < pars.ds) {
            sepImp[0] += x - nearBoids[i].x;
            sepImp[1] += y - nearBoids[i].y;
        }
    }
    sepImp[0] *= pars.s;
    sepImp[1] *= pars.s;
    return sepImp;
}

std::array<double, 2> Boid::allignmentImpulse (SimulationParameters pars) {

    double sumVelX{ 0 };
    double sumVelY{ 0 };

    for (int i = 0; i < (int)size (nearBoids); ++i) {
        sumVelX += nearBoids[i].velX;
        sumVelY += nearBoids[i].velY;
    }

    return { pars.s * sumVelX / size (nearBoids), pars.s * sumVelY / size (nearBoids) };
}


std::array<double, 2> Boid::cohesionImpulse (SimulationParameters pars) {

    double sumX{ 0 };
    double sumY{ 0 };

    for (int i = 0; i < (int)size (nearBoids); ++i) {
        sumX += nearBoids[i].x;
        sumY += nearBoids[i].y;
    }

    return { (pars.c * sumX / size (nearBoids)), (pars.c * sumY / size (nearBoids)) };
}

void Boid::updatePosition () {
    x += velX;
    y += velY;

    // torus-like space
    while (x > X_SPACE) {
        x -= X_SPACE;
    };
    while (y > Y_SPACE) {
        y -= Y_SPACE;
    };
    while (x < 0) {
        x += X_SPACE;
    };
    while (y < 0) {
        y += Y_SPACE;
    };
}

void Boid::updateVelocity () {

    velX += impX;
    velY += impY;

    // enforcing the speed limit
    if (velX > MAX_COMPONENT_SPEED) {
        velX = MAX_COMPONENT_SPEED;
    }
    if (velX < -MAX_COMPONENT_SPEED) {
        velX = -MAX_COMPONENT_SPEED;
    }
    if (velY > MAX_COMPONENT_SPEED) {
        velY = MAX_COMPONENT_SPEED;
    }
    if (velY < -MAX_COMPONENT_SPEED) {
        velY = -MAX_COMPONENT_SPEED;
    }
}

void Boid::updateImpulse (SimulationParameters pars) {
    std::array<double, 2> sepImp = separationImpulse (pars);
    std::array<double, 2> allImp = allignmentImpulse (pars);
    std::array<double, 2> coImp  = cohesionImpulse (pars);
    impX                         = sepImp[0] + allImp[0] + coImp[0];
    impY                         = sepImp[1] + allImp[1] + coImp[1];
}

void Boid::updateNeighbours (const std::vector<Boid>& state, SimulationParameters pars) {
    nearBoids.clear ();
    nearBoidsDist.clear ();

    for (auto b : state) {
        if (&b == this) {
            continue;
        }
        double dist = this->distance (b);

        if (dist < pars.d) {
            nearBoids.push_back (b);
            nearBoidsDist.push_back (dist);
        }
    }
}
// [0] is the x coordinate and [1] is the y

double Boid::distance (const Boid& b) {
    return std::sqrt (std::pow (this->x - b.x, 2) + std::pow (this->y - b.y, 2));
}
