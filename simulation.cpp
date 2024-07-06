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
    FlightStatistics stats{gatherData()};
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

FlightStatistics Simulation::gatherData() const
{
  float meanDist{0.f};
  float meanSquaredDist{0.f};
  ArrayF2 meanVel{0.f, 0.f};
  ArrayF2 meanSquaredVel{0.f, 0.f};

  for (auto it = flock.begin(); it != flock.end(); ++it) {
    ArrayF2 vel{it->getVelocity()};
    meanVel += vel;
    meanSquaredVel += {std::pow(vel[0], 2.f), std::pow(vel[1], 2.f)};

    if (parameters.boidNumber < 2)
        continue;
    for (auto jt = it + 1; jt != flock.end(); ++jt) {
      float dist = euclidianNorm(it->getPosition() - jt->getPosition());
      meanDist += dist;
      meanSquaredDist += std::pow(dist, 2.f);
    }
  }
  // combinations without repetitions of flock.size() elements
  float nDist{static_cast<float>(parameters.boidNumber * ( parameters.boidNumber - 1))};
  meanDist /= nDist;
  meanSquaredDist /= nDist;
  meanVel /= parameters.boidNumber;
  meanSquaredVel /= parameters.boidNumber;
  float varianceDist{meanSquaredDist - std::pow(meanDist, 2.f)};
  float stdDevDist{std::sqrt(varianceDist)};
  ArrayF2 varianceVel{
      meanSquaredVel
      - ArrayF2{std::pow(meanVel[0], 2.f), std::pow(meanVel[1], 2.f)}};
  ArrayF2 stdDevVel{std::sqrt(varianceVel[0]), std::sqrt(varianceVel[1])};
  return {meanDist, stdDevDist, meanVel, stdDevVel};
}
