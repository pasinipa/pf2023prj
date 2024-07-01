#pragma once
#include "statistics.hpp"
#include "arrayop.hpp"
#include <array>
#include <cstddef>
#include <numeric>
#include <cmath>

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

  for (int t{0}; t < time; t += 5) {
    double mean_distN{0};
    for (int i{0}; i < std::size(state); ++i) {
      for (int j{i + 1}; j <= std::size(state); ++j) {
        double distance =
            euclidianNorm(state[i].m_position - state[j].m_position);
        mean_distN += distance;
      }
    }

    std::cout << "The mean distance at time " << t << " is "
              << mean_distN / std::size(state) << '\n';
  }
}

auto Statistics::Mean_velocity(int time, std::vector<Boid> const& state)
{
  /*
    double mean_velN{0};
    auto velocity;
    for (time % 10 = 0) {
      for (int i = 0; i < std::size(state); i++) {
        velocity                                       = state.m_velocity[i];
        oppure i lo passo by reference ? (*i)mean_velN =
    +euclidianNorm(velocity);
      }

      float mean_Vel = mean_velN / std::size(state);

      cout << " the mean velocity at time " << time << " is " << mean_Vel;
    }
  */
  double mean_Vel;

  for (int t{0}; t < time; t += 5) 
  {
    double mean_velN{0};
    for (auto b : state) 
    {
      double deltaV = euclidianNorm(b.m_velocity);
      mean_velN += deltaV;
    }

    mean_Vel = mean_velN / std::size(state);

    std::cout << "The mean velocity at time " << t << " is "
              << mean_Vel << '\n';
  }

return mean_Vel;

}

void Statistics::Sdeviation_vel(int time, std::vector<Boid> const& state)
{
  auto Sdev_V;
  auto vel_media = Statistics::Mean_velocity();
  double sum_quadrati = 0;
  for (int t{0}; t < time; t += 5) 
  {
    for (auto b : state) 
    {
      sum_quadrati = sum_quadrati + std::pow(vel_media - euclidianNorm(b.m_velocity),2); //pow richiede 2 argomenti prima base, poi esponente
    }

  Sdev_V = std::sqrt(sum_quadrati/(std::size(state)-1));

  }

  std::cout << "The Standard deviation of the velocity at time " << t << " is "
              << Sdev_V << '\n';
}

void Statistics::Sdeviation_vel(int time, std::vector<Boid> const& state)
{
  auto Sdev_D;
  auto dist_media = Statistics::Mean_distance();
  double sum_quadrati = 0;
  double N_tot_dev = 0; //N al denominatore della deviazione standard

  for (int t{0}; t < time; t += 5)
  {
    for (int i{0}; i < std::size(state); ++i) 
    {
      for (int j{i + 1}; j <= std::size(state); ++j) 
      {
        sum_quadrati = sum_quadrati + std::pow((dist_media - euclidianNorm(state[i].m_position - state[j].m_position)),2);

        N_tot_dev += 1;
      }

      N_tot_dev += 1;
    }
  Sdev_D = std::sqrt(sum_quadrati/(N_tot_dev-1));

  //Sdev_D = std::sqrt(sum_quadrati/((std::size(state)*(std::size(state)-1)-1))); //quante coppie posso fare con N elementi senza ripetiziono

  }
  std::cout << "The Standard deviation of the distance at time " << t << " is "
              << Sdev_D << '\n';
}