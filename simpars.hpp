#pragma once

struct SimPars
{
  float ds{1.f};
  float d{20.f};
  float s{5.f};
  float a{0.1f};
  float c{1.f};
  int bNum{10};
  bool isVerbose{false};
  double w{}; //wall's impulse

  SimPars() = default;
};
