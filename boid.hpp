#include <vector>

// const double SIGHT_DIST = 10;

class Boid {

    // using an array instead of three unbounded vars makes it easy to compute distance
    // with std::inner_product()
    std::array<double, 3> position; 
    std::array<double, 3> velocity; 
    std::array<double, 3> impulse; 

    // it should always be that size(nearBoids) == size(nearBoidsDistances)
    std::vector<Boid*> nearBoids; 
    std::vector<double> nearBoidsDistances; 


    double separationImpulse ();
    double allignmentImpulse ();
    double cohesionImpulse ();

    public:
    void updatePosition ();
    void updateVelocity ();
    void updateImpulse ();
    void updateNeighbours (std::vector<Boid*> allBoids);

};

double distance(const& Boid, const& Boid);
