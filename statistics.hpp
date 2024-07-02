#pragma once
#include "boid.hpp"

class Statistics 
{
    double Mean_distance(int time, std::vector<Boid> const& state);
    double Mean_velocity(int time, std::vector<Boid> const& state);
    void Sdeviation_vel(int time, std::vector<Boid> const& state);
    void Sdeviation_dis(int time, std::vector<Boid> const& state);
};