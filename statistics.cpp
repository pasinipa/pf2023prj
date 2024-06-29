#pragma once
#include "statistics.hpp"
#include <array>
#include <cstddef>

void Statistics::Mean_distance(int time, std::vector<Boid> const& state)
{
    /* double mean_distN {0};
    for ( time%10 = 0) 
    {
        for (int i = 0; i < std::size(state); i++) 
        {
            for(int j = i + 1; j < std::size(state) ;j++)
            {
                state[i(m_position)] - state[j(m_position)] = distance;      oppure passo sia i che j by reference?
                mean_distN =+ euclidianNorm(distance);
            }
        }
    
    float mean_Dist = mean_distN / std::size(state);

    cout << " the mean distance at time " << time << " is " << mean_Dist;
    
    }    
    */
}

void Statistics::Mean_velocity(int time, std::vector<Boid> const& state)
{
        /* double mean_velN {0};
    for ( time%10 = 0) 
    {
        for (int i = 0; i < std::size(state); i++) 
        {

        state[i(m_velocity)] = velocity;        oppure i lo passo by reference? (*i)
        mean_velN =+ euclidianNorm(velocity);
        
        }
    
    float mean_Vel = mean_velN / std::size(state);

    cout << " the mean velocity at time " << time << " is " << mean_Vel;
    }    
    */
}

void Statistics::Sdeviation(int time, std::vector<Boid> const& state)
{}
