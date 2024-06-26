#pragma once

inline std::array<double, 2> operator+(std::array<double, 2> const& a1,
                                       std::array<double, 2> const& a2)
{
  return {a1[0] + a2[0], a1[1] + a2[1]};
}

inline std::array<double, 2> operator-(std::array<double, 2> const& a1,
                                       std::array<double, 2> const& a2)
{
  return {a1[0] - a2[0], a1[1] - a2[1]};
}

inline std::array<double, 2> operator/(std::array<double, 2> const& a1,
                                       double d)
{
  return {a1[0] / d, a1[1] / d};
}

inline std::array<double, 2> operator*(std::array<double, 2> const& a1,
                                       double d)
{
  return {a1[0] * d, a1[1] * d};
}

inline void operator+=(std::array<double, 2>& a1,
                       std::array<double, 2> const& a2)
{
  a1[0] += a2[0];
  a1[1] += a2[1];
}

inline void operator/=(std::array<double, 2>& a1, double d)
{
  a1[0] /= d;
  a1[1] /= d;
}

inline void operator*=(std::array<double, 2>& a1, double d)
{
  a1[0] *= d;
  a1[1] *= d;
}
