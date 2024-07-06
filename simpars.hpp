#pragma once

struct SimPars
{
  int boidNumber{200};
  int sampleRate{50};
  float perceptionRadius{40.f};
  float separationRuleRadius{15.f};
  float separationRuleStrength{15.f};
  float allignmentRuleStrength{1.f};
  float cohesionRuleStrength{0.01f};
  float obstacleRuleRadius{80.f};
  float obstacleRuleStrength{3.f};
  bool isVerbose{false};

  SimPars() = default;
};
