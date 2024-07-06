#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "arrayop.hpp"
#include "boid.hpp"

#include "doctest.h"

TEST_CASE("testing array operations")
{
  ArrayF2 a1{1., 3.};
  ArrayF2 a2{-2., 0.};
  float k = 2.;

  auto sum  = a1 + a2;
  auto diff = a1 - a2;
  auto prod = a1 * k;
  auto quot = a2/k; // a voler essere precisi bisognerebbe escludere che divida per 0...

  CHECK(sum == (-1., 3.));
  CHECK(diff == (3., 3.));
  CHECK(prod == (2., 6.));
  CHECK(quot == (-1, 0.));

  a1 += a2;
  a1 *= k;
  a2 /= k;
  CHECK(a1 == (-2., 6.));
  CHECK(a2 == (-1, .0));
}

TEST_CASE("testing simulation class members") {
    
}

TEST_CASE("testing boid class members") {
    Boid b1;
    Boid b2;
    Boid b3;
/*
    b1.m_position = {};
    b1.m_velocity = {};
    b2.m_position = {};
    b2.m_velocity = {};
    b3.m_position = {};
    b3.m_velocity = {};

    std::vector<Boid> Tstate{b1, b2, b3};
    for (auto& b : Tstate) b.updateNeighbourhood(Tstate);
*/

    

}

TEST_CASE("testing statistics class members") {

}