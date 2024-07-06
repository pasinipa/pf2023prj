#include "simulation.hpp"
#include "arrayop.hpp"
#include <cmath>
#include <iomanip>
#include <vector>

Simulation::Simulation()
    : flock{std::vector<Boid>(
        static_cast<unsigned long>(parameters.boidNumber))}
    , obstacles{std::vector<Obstacle>(static_cast<unsigned long>(parameters.obstacleNumber))}
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

void Simulation::updateView(sf::RenderWindow& window) const
{
  sf::CircleShape triangle(5.f, 3);
  sf::CircleShape circle(5.f);
  circle.setFillColor(sf::Color::Red);

  for (const auto& b : flock) {
    auto [x, y] = b.getPosition();
    triangle.setPosition(x, y);
    window.draw(triangle);
  }
  for (auto const& o : obstacles) {
    auto [x, y] = o.position;
    circle.setPosition(x, y);
    window.draw(circle);
  }
}

void Simulation::streamStatsToFile(std::ofstream& outputFile) const
{
  if (time % parameters.sampleRate == 0 and outputFile.is_open()) {
    FlightStatistics stats{flightData()};
    outputFile << std::fixed << std::setprecision(3) << std::setfill(' ');
    outputFile << "| " << std::setw(6) << time;
    outputFile << " | " << std::setw(6) << stats.meanDist;
    outputFile << " | " << std::setw(6) << stats.stdDevDist;
    outputFile << " | " << std::setw(6) << stats.meanVel[0];
    outputFile << " | " << std::setw(6) << stats.meanVel[1];
    outputFile << " | " << std::setw(6) << stats.stdDevVel[0];
    outputFile << " | " << std::setw(6) << stats.stdDevVel[1];
    outputFile << " |" << std::endl;
    
  }
}

float euclidianNorm(ArrayF2 const&);

FlightStatistics Simulation::flightData() const
{
  float meanDist{0.f};
  float meanSquaredDist{0.f};
  ArrayF2 meanVel{0.f, 0.f};
  ArrayF2 meanSquaredVel{0.f, 0.f};

  for (auto it = flock.begin(); it != flock.end(); ++it) {
    for (auto jt = it + 1; jt != flock.end(); ++jt) {
      float dist = euclidianNorm(it->getPosition() - jt->getPosition());
      meanDist += dist;
      meanSquaredDist += std::powf(dist, 2);
    }
    ArrayF2 vel{it->getVelocity()};
    meanVel += vel;
    meanSquaredVel += {std::powf(vel[0], 2), std::powf(vel[1], 2)};
  }
  // combinations without repetitions of flock.size() elements
  float N{static_cast<float>(flock.size() * (flock.size() - 1))};
  meanDist /= N;
  meanSquaredDist /= N;
  meanVel /= parameters.boidNumber;
  meanSquaredVel /= parameters.boidNumber;
  float varianceDist{meanSquaredDist - std::powf(meanDist, 2)};
  float stdDevDist{std::sqrt(varianceDist)};
  ArrayF2 varianceVel{
      meanSquaredVel
      - ArrayF2{std::powf(meanVel[0], 2), std::powf(meanVel[1], 2)}};
  ArrayF2 stdDevVel{std::sqrt(varianceVel[0]), std::sqrt(varianceVel[1])};
  return {meanDist, stdDevDist, meanVel, stdDevVel};
}
