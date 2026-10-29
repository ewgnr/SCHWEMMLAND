#include "Swarm9D.h"

#include <cmath>
#include <random>
#include <algorithm>

// Limit vector magnitude
static Vec9 limitVector(const Vec9 & v, double maxLength)
{
	double length = v.norm();

	if (length > maxLength && length > 0.000001)
	{
		return v * (maxLength / length);
	}

	return v;
}

void Swarm9D::setup(int count)
{
	boids.clear();
	boids.reserve(count);

	std::random_device rd;
	std::mt19937 generator(rd());

	std::normal_distribution<double> distribution(0.0, 1.0);
	std::uniform_real_distribution<double> radiusDistribution(0.0, 1.0);

	std::uniform_real_distribution<double> sepDist(0.7, 1.8);
	std::uniform_real_distribution<double> aliDist(0.5, 1.5);
	std::uniform_real_distribution<double> cohDist(0.5, 1.5);

	constexpr double initialRadius = 25.0;

	for (int i = 0; i < count; i++)
	{
		Boid9D boid;

		Vec9 direction;

		for (int d = 0; d < 9; d++)
		{
			direction[d] = distribution(generator);
		}

		direction.normalize();

		// Uniform volume distribution in a 9D ball
		double radius = initialRadius * std::pow(radiusDistribution(generator), 1.0 / 9.0);

		boid.position = direction * radius;

		// Random initial velocity
		for (int d = 0; d < 9; d++)
		{
			direction[d] = distribution(generator);
		}

		direction.normalize();

		boid.velocity = direction * boid.maxSpeed;
		boid.separationWeight = sepDist(generator);
		boid.alignmentWeight = aliDist(generator);
		boid.cohesionWeight = cohDist(generator);

		boids.push_back(boid);
	}
}

Vec9 Swarm9D::containment(const Boid9D & boid) const
{
	double distance = boid.position.norm();

	if (distance <= containmentRadius)
		return Vec9::Zero();

	// Distance beyond the containment boundary
	double excess = distance - containmentRadius;

	// Gradually increase containment strength
	double strength = std::clamp(excess / 20.0, 0.0, 1.0);

	Vec9 desired = -boid.position;

	return steerTowards(boid, desired) * strength;
}

Vec9 Swarm9D::separation(const Boid9D & boid) const
{
	Vec9 steering = Vec9::Zero();
	int count = 0;

	for (const auto & other : boids)
	{
		if (&boid == &other)
			continue;

		Vec9 difference = boid.position - other.position;

		double distance = difference.norm();

		if (distance > 0.000001 && distance < separationRadius)
		{
			steering += difference / (distance * distance);

			count++;
		}
	}

	if (count > 0)
	{
		steering /= count;

		return steerTowards(boid, steering);
	}

	return Vec9::Zero();
}

Vec9 Swarm9D::alignment(const Boid9D & boid) const
{
	Vec9 averageVelocity = Vec9::Zero();
	int count = 0;

	for (const auto & other : boids)
	{
		if (&boid == &other)
			continue;

		double distance = (boid.position - other.position).norm();

		if (distance < alignmentRadius)
		{
			averageVelocity += other.velocity;
			count++;
		}
	}

	if (count > 0)
	{
		averageVelocity /= count;

		return steerTowards(boid, averageVelocity);
	}

	return Vec9::Zero();
}

Vec9 Swarm9D::cohesion(const Boid9D & boid) const
{
	Vec9 center = Vec9::Zero();
	int count = 0;

	for (const auto & other : boids)
	{
		if (&boid == &other)
			continue;

		double distance = (boid.position - other.position).norm();

		if (distance < cohesionRadius)
		{
			center += other.position;
			count++;
		}
	}

	if (count > 0)
	{
		center /= count;

		Vec9 desired = center - boid.position;

		return steerTowards(boid, desired);
	}

	return Vec9::Zero();
}

Vec9 Swarm9D::steerTowards(const Boid9D & boid, const Vec9 & desired) const
{
	if (desired.norm() < 0.000001)
		return Vec9::Zero();

	Vec9 desiredVelocity = desired.normalized() * boid.maxSpeed;

	Vec9 steering = desiredVelocity - boid.velocity;

	return limitVector(steering, boid.maxForce);
}

Vec9 Swarm9D::getCenter() const
{
	Vec9 center = Vec9::Zero();

	if (boids.empty())
		return center;

	for (const auto & boid : boids)
	{
		center += boid.position;
	}

	return center / static_cast<double>(boids.size());
}

double Swarm9D::getAverageRadius() const
{
	if (boids.empty())
		return 0.0;

	Vec9 center = getCenter();

	double totalDistance = 0.0;

	for (const auto & boid : boids)
	{
		totalDistance += (boid.position - center).norm();
	}

	return totalDistance / static_cast<double>(boids.size());
}

double Swarm9D::getAverageSpeed() const
{
	if (boids.empty())
		return 0.0;

	double total = 0.0;

	for (const auto & boid : boids)
	{
		total += boid.velocity.norm();
	}

	return total / static_cast<double>(boids.size());
}

double Swarm9D::getVelocityDispersion() const
{
	if (boids.empty())
		return 0.0;

	Vec9 meanVelocity = Vec9::Zero();

	for (const auto & boid : boids)
	{
		meanVelocity += boid.velocity;
	}

	meanVelocity /= static_cast<double>(boids.size());

	double variance = 0.0;

	for (const auto & boid : boids)
	{
		variance += (boid.velocity - meanVelocity).squaredNorm();
	}

	return std::sqrt(variance / static_cast<double>(boids.size()));
}

Vec9 Swarm9D::getMeanVelocity() const
{
	Vec9 meanVelocity = Vec9::Zero();

	if (boids.empty())
		return meanVelocity;

	for (const auto & boid : boids)
	{
		meanVelocity += boid.velocity;
	}

	meanVelocity /= static_cast<double>(boids.size());

	return meanVelocity;
}

void Swarm9D::update(double dt)
{
	// First calculate all forces.
	// All boids still have their previous positions.
	for (auto & boid : boids)
	{
		Vec9 sep = separation(boid);
		Vec9 ali = alignment(boid);
		Vec9 coh = cohesion(boid);
		Vec9 con = containment(boid);

		Vec9 totalForce = sep * boid.separationWeight * separationWeight + ali * boid.alignmentWeight * alignmentWeight + coh * boid.cohesionWeight * cohesionWeight + con * containmentWeight;

		boid.acceleration = limitVector(totalForce, boid.maxForce);
	}

	// Then update all boids.
	for (auto & boid : boids)
	{
		boid.velocity += boid.acceleration * dt;

		boid.velocity = limitVector(boid.velocity, boid.maxSpeed);

		boid.position += boid.velocity * dt;

		boid.acceleration.setZero();
	}
}
