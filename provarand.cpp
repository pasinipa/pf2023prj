#include <iostream>
#include <random>

using ArrayD2 = std::array<double, 2>;

int main()
{
  std::default_random_engine engPos;
  std::normal_distribution<double> dist1{50., 15.};
  double a = dist1(engPos);
  double b = dist1(engPos);

  std::default_random_engine engVel;
  std::normal_distribution<double> dist2{0.5, 0.2};
  double c = dist2(engVel);
  double d = dist2(engVel);

  std::cout << a << ' ' << b << ' ' << c << ' ' << d;
}
