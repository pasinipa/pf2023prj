#include "simulation.hpp"
#include <vector>

Simulation::Simulation()
    : state{std::vector<Boid>(static_cast<unsigned long> (Simulation::parameters.bNum))}
{}

void Simulation::updateState()
{
  for (auto& b : state) {
    b.updateImpulse(state);
    //some way to determine the closest (if not the only) wall, say w1
    b.wallDeviation(wallsConfig);
    b.edgeBounce();
  }
  for (auto& b : state) {
    b.updatePosition();
    b.updateVelocity();
  }
  ++time;
}

void Simulation::updateView(sf::RenderWindow& window)
{
  // sf::VertexArray triangle(sf::Triangles, 3);
  sf::CircleShape triangle(5.f, 3);
  for (const auto& b : state) {
    auto [x, y] = b.getPosition();
    triangle.setPosition(x, y);
    // triangle[0].position = sf::Vector2f ( 400.f , 400.f );
    // triangle[1].position = sf::Vector2f { x - 5.f, y + 3.f };
    // triangle[0].position = sf::Vector2f { x - 5.f, y - 3.f };
    window.draw(triangle);
  }
}
