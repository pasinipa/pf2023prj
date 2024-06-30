#pragma once
#include "statistics.hpp"
#include "arrayop.hpp"
#include <array>
#include <cstddef>
#include <numeric>

double euclidianNorm(ArrayD2 const& arr)
{
  double norm{0};
  norm = std::inner_product(arr.begin(), arr.end(), arr.begin(), 0.);
  norm = std::sqrt(norm);
  return norm;
}

void Statistics::Mean_distance(int time, std::vector<Boid> const& state)
{
  /*
      double mean_distN {0};
      auto distance;
      for ( time%10 = 0)
      {
          for (int i = 0; i < std::size(state); i++)
          {
              for (int j = i + 1; j < std::size(state) ;j++)
              {
                  distance = state.m_position[i] - state.m_position[j]; //oppure
     passo sia i che j by reference? mean_distN =+ euclidianNorm(distance);
              }
          }

      float mean_Dist = mean_distN / std::size(state);

      cout << " the mean distance at time " << time << " is " << mean_Dist;

      }
  */

  double mean_distN{0};
  for (auto b : state) {
    for (auto c : state) {
      if (&c == &b) {
        continue;
      }

      double distance = euclidianNorm(b.m_position - c.m_position);
      mean_distN += distance;
    }
  }

  std::cout << "The mean distance at time " << time << " is "
            << mean_distN / std::size(state) << '\n';
}

void Statistics::Mean_velocity(int time, std::vector<Boid> const& state)
{
/*    
  double mean_velN{0};
  auto velocity;
  for (time % 10 = 0) {
    for (int i = 0; i < std::size(state); i++) {
      velocity                                       = state.m_velocity[i];
      oppure i lo passo by reference ? (*i)mean_velN = +euclidianNorm(velocity);
    }

    float mean_Vel = mean_velN / std::size(state);

    cout << " the mean velocity at time " << time << " is " << mean_Vel;
  }
*/

double mean_velN{0};
for (auto b : state) {
      double deltaV = euclidianNorm(b.m_velocity);
      mean_velN += deltaV;
    }

  std::cout << "The mean velocity at time " << time << " is "
            << mean_velN / std::size(state) << '\n';    

}

void Statistics::Sdeviation(int time, std::vector<Boid> const& state)
{}
