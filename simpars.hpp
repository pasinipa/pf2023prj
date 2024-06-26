#pragma once

struct SimPars
{
  double ds{1};
  double d{20};
  double s{5};
  double a{0.7};
  double c{0.3};
  int bNum{10};
  bool isVerbose{false};

  SimPars() = default;
};
