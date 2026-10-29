#pragma once

#include <Eigen/Dense>

using Vec9 = Eigen::Matrix<double, 9, 1>;

struct Boid9D
{
	Vec9 position = Vec9::Zero();
	Vec9 velocity = Vec9::Zero();
	Vec9 acceleration = Vec9::Zero();

	double maxSpeed = 15.0;
	double maxForce = 20.0;

	double separationWeight = 1.5;
	double alignmentWeight = 0.35;
	double cohesionWeight = 0.65;
};
