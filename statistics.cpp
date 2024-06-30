#pragma once
#include "statistics.hpp"
#include <array>
#include <cstddef>

void Statistics::Mean_distance(time, std::vector<Boid> const& state)
{
    double mean_distN {0};
    auto distance;
    for ( time%10 = 0) 
    {
        for (int i = 0; i < std::size(state); i++) 
        {
            for (int j = i + 1; j < std::size(state) ;j++)
            {
                distance = state.m_position[i] - state.m_position[j];      //oppure passo sia i che j by reference?
                mean_distN =+ euclidianNorm(distance);
            }
        }
    
    float mean_Dist = mean_distN / std::size(state);

    cout << " the mean distance at time " << time << " is " << mean_Dist;
    
    }    
    
}

void Statistics::Mean_velocity(time, std::vector<Boid> const& state)
{
    double mean_velN {0};
    auto velocity;
    for ( time%10 = 0) 
    {
        for (int i = 0; i < std::size(state); i++) 
        {

        velocity = state.m_velocity[i];        oppure i lo passo by reference? (*i)
        mean_velN =+ euclidianNorm(velocity);
        
        }
    
    float mean_Vel = mean_velN / std::size(state);

    cout << " the mean velocity at time " << time << " is " << mean_Vel;
    }    

}

void Statistics::Sdeviation(time, std::vector<Boid> const& state)
{}
