#pragma once
#include "statistics.hpp"
#include "arrayop.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <numeric>

double euclidianNorm(ArrayD2 const& arr)
{
  double norm{0};
  norm = std::inner_product(arr.begin(), arr.end(), arr.begin(), 0.);
  norm = std::sqrt(norm);
  return norm;
}

double Statistics::Mean_distance(int time, std::vector<Boid> const& state)
{
  double mean_dist{0};

  for (int t{0}; t < time; t += 5) {
    double mean_distN{0};
    for (int i{0}; i < std::size(state); ++i) {
      for (int j{i + 1}; j <= std::size(state); ++j) {
        double distance =
            euclidianNorm(state[i].m_position - state[j].m_position);
        mean_distN += distance;
      }
    }
  }

  return mean_dist;
}

double Statistics::Mean_velocity(int time, std::vector<Boid> const& state)
{
  double mean_vel{0};

  for (int t{0}; t < time; t += 5) {
    double mean_velN{0};

    for (auto b : state) {
      double deltaV = euclidianNorm(b.m_velocity);
      mean_velN += deltaV;
    }

    mean_vel = mean_velN / std::size(state);
  }

  return mean_vel;
}

double Statistics::Sdeviation_vel(int time, std::vector<Boid> const& state)
{
  double Sdev_V{0};
  auto vel_media = Statistics::Mean_velocity(time, state);
  double sum_quadrati{0};

  for (int t{0}; t < time; t += 5) {
    for (auto b : state) {
      sum_quadrati =
          sum_quadrati
          + std::pow(vel_media - euclidianNorm(b.m_velocity),
                     2); // pow richiede 2 argomenti prima base, poi esponente
    }

    Sdev_V = std::sqrt(sum_quadrati / (std::size(state) - 1));
  }

  return Sdev_V;
}

double Statistics::Sdeviation_dis(int time, std::vector<Boid> const& state)
{
  double Sdev_D{0};
  double dist_media = Statistics::Mean_distance(time, state);
  double sum_quadrati{0};
  double N_tot_dev{0}; // N al denominatore della deviazione standard

  for (int t{0}; t < time; t += 5) {
    for (int i{0}; i < std::size(state); ++i) {
      for (int j{i + 1}; j <= std::size(state); ++j) {
        sum_quadrati = sum_quadrati
                     + std::pow((dist_media
                                 - euclidianNorm(state[i].m_position
                                                 - state[j].m_position)),
                                2);

        N_tot_dev += 1;
      }
      N_tot_dev += 1;
    }

    Sdev_D = std::sqrt(sum_quadrati / (N_tot_dev - 1));

    // Sdev_D =
    // std::sqrt(sum_quadrati/((std::size(state)*(std::size(state)-1)-1)));
    // //quante coppie posso fare con N elementi senza ripetiziono
  }

  return Sdev_D;
}