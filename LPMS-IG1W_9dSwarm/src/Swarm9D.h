#pragma once

#include "Boid9D.h"
#include <vector>

class Swarm9D
{
public:
	std::vector<Boid9D> boids;

	double separationWeight = 1.8;
	double alignmentWeight = 0.35;
	double cohesionWeight = 0.65;

	double separationRadius = 12.0;
	double alignmentRadius = 18.0;
	double cohesionRadius = 22.0;

	double containmentRadius = 18.0;
	double containmentWeight = 0.5;

	void setup(int count);
	void update(double dt);

	Vec9 getCenter() const;
	double getAverageRadius() const;
	double getAverageSpeed() const;
	double getVelocityDispersion() const;
	Vec9 getMeanVelocity() const;

private:
	Vec9 separation(const Boid9D & boid) const;
	Vec9 alignment(const Boid9D & boid) const;
	Vec9 cohesion(const Boid9D & boid) const;

	Vec9 steerTowards(const Boid9D & boid, const Vec9 & desired) const;

	Vec9 containment(const Boid9D & boid) const;
};
