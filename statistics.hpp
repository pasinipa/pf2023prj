#pragma once
#include "boid.hpp"

class Statistics 
{
    void Mean_distance(int time, std::vector<Boid> const& state);
    void Mean_velocity(int time, std::vector<Boid> const& state);
    void Sdeviation(int time, std::vector<Boid> const& state);
};