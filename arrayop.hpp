#pragma once
using ArrayF2 = std::array<float, 2>;

inline ArrayF2 operator+(ArrayF2 const& a1, ArrayF2 const& a2)
{
  return {a1[0] + a2[0], a1[1] + a2[1]};
}

inline ArrayF2 operator-(ArrayF2 const& a1, ArrayF2 const& a2)
{
  return {a1[0] - a2[0], a1[1] - a2[1]};
}

inline ArrayF2 operator/(ArrayF2 const& a1, float f)
{
  if (f != 0.f)
    return {a1[0] / f, a1[1] / f};

  return a1;
}

inline ArrayF2 operator*(ArrayF2 const& a1, float f)
{
  return {a1[0] * f, a1[1] * f};
}

inline void operator+=(ArrayF2& a1, ArrayF2 const& a2)
{
  a1[0] += a2[0];
  a1[1] += a2[1];
}

inline void operator/=(ArrayF2& a1, float f)
{
  if (f != 0.f) { 
    a1[0] /= f;
    a1[1] /= f;
  }
}

inline void operator*=(ArrayF2& a1, float f)
{
  a1[0] *= f;
  a1[1] *= f;
}
