#include "simulation.hpp"
#include <vector>


Simulation::Simulation()
    : flock{std::vector<Boid>(
        static_cast<unsigned long>(parameters.boidNumber))}
    , obstacles{{300.f, 500.f}}
{}

void Simulation::updateState()
{
  for (auto& b : flock) {
    b.updateImpulse(flock, obstacles);
  }
  for (auto& b : flock) {
    b.updatePosition();
    b.updateVelocity();
  }
  ++time;
}

void Simulation::updateView(sf::RenderWindow& window)
{
  // sf::VertexArray triangle(sf::Triangles, 3);
  sf::CircleShape triangle(5.f, 3);
  sf::CircleShape circle(5.f);
  circle.setFillColor(sf::Color::Red);
  for (const auto& b : flock) {
    auto [x, y] = b.getPosition();
    triangle.setPosition(x, y);
    // triangle[0].position = sf::Vector2f ( 400.f , 400.f );
    // triangle[1].position = sf::Vector2f { x - 5.f, y + 3.f };
    // triangle[0].position = sf::Vector2f { x - 5.f, y - 3.f };
    window.draw(triangle);
  }
  for (auto const& o : obstacles) {
    auto [x, y] = o.position;
    circle.setPosition(x, y);
    window.draw(circle);
  }
}
