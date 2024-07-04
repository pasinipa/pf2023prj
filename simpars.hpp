#pragma once

struct SimPars
{
  float ds{15.f};
  float d{40.f};
  float s{15.f};
  float a{1.f};
  float c{0.01f};
  int bNum{200};
  bool isVerbose{false};
  double w{}; // wall's impulse

  SimPars() = default;
};
