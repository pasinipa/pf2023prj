#pragma once
#include <stdexcept>

struct SimPars
{
  int boidNumber{200};
  int sampleRate{50};
  float perceptionRadius{40.f};
  float separationRadius{15.f};
  float separationStrength{15.f};
  float allignmentStrength{1.f};
  float cohesionStrength{0.01f};
  float obstacleRadius{80.f};
  float obstacleStrength{3.f};
  bool boundariesEnabled{false};

  inline void checkValidity() {
    if (boidNumber < 0
            or sampleRate < 0
            or perceptionRadius < 0.f
            or separationRadius < 0.f
            or separationStrength < 0.f
            or allignmentStrength < 0.f
            or cohesionStrength < 0.f
            or obstacleRadius < 0.f
            or obstacleStrength < 0.f)
        throw std::invalid_argument {"check for any negative values."};
  }

  SimPars() = default;
};
