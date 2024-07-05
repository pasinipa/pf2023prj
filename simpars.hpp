#pragma once

struct SimPars
{
  int boidNumber{200};
  float perceptionRadius{40.f};
  float separationRuleRadius{15.f};
  float separationRuleStrength{15.f};
  float allignmentRuleStrength{1.f};
  float cohesionRuleStrength{0.01f};
  float obstacleRuleRadius{0.f};
  float obstacleRuleStrength{0.f};
  bool isVerbose{false};
  bool edges{false};

  SimPars() = default;
};
