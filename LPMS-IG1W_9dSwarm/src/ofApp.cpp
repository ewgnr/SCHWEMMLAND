#include "ofApp.h"

#include <cmath>
#include <cstring>
#include <algorithm>

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetFrameRate(60);
	ofBackground(14, 17, 23);

	ofEnableDepthTest();

	swarm.setup(8);

	oscSender.setup(oscHost, oscPort);

	cam.setDistance(120);

	projection << 1.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.2, 0.0, 0.0,
				  0.0, 1.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.2, 0.0,
				  0.0, 0.0, 1.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.2;

	udp.Create();
	udpInitialized = udp.Bind(9002);
	udp.SetNonBlocking(true);

	if (udpInitialized)
	{
		ofLogNotice("9DSwarm") << "Listening for LPMS UDP on port 9003";
	}
	else
	{
		ofLogError("9DSwarm") << "Could not bind UDP port 9003";
	}

	baseSeparationWeight = swarm.separationWeight;
	baseAlignmentWeight = swarm.alignmentWeight;
	baseCohesionWeight = swarm.cohesionWeight;
}

void ofApp::receivePackets()
{
	if (!udpInitialized)
		return;

	char buffer[1024];

	// Drain queued packets and keep the newest valid one.
	for (int i = 0; i < 100; ++i)
	{
		int n = udp.Receive(buffer, sizeof(buffer));

		if (n <= 0)
			break;

		if (n != static_cast<int>(sizeof(ImuPacket)))
			continue;

		ImuPacket packet {};
		std::memcpy(&packet, buffer, sizeof(packet));

		const float values[] = {
			packet.qw, packet.qx, packet.qy, packet.qz,
			packet.wx, packet.wy, packet.wz,
			packet.ax, packet.ay, packet.az
		};

		bool valid = true;

		for (float value : values)
		{
			if (!std::isfinite(value))
			{
				valid = false;
				break;
			}
		}

		if (!valid)
			continue;

		const float qNormSquared = packet.qw * packet.qw + packet.qx * packet.qx + packet.qy * packet.qy + packet.qz * packet.qz;

		if (!std::isfinite(qNormSquared) || qNormSquared < 1e-6f)
			continue;

		sensorData = packet;
		hasSensorData = true;
	}
}


void ofApp::updateSensorControls()
{
	if (!hasSensorData)
		return;

	// Smooth the controls to reduce sensor noise.
	// Assumes update() runs at approximately 60 Hz.
	constexpr double smoothing = 0.12;

	// 1. Linear acceleration magnitude (units: g)
	const double ax = sensorData.ax;
	const double ay = sensorData.ay;
	const double az = sensorData.az;

	const double accelMagnitude = std::sqrt(ax * ax + ay * ay + az * az);

	// 2. Angular velocity magnitude (units: degrees/second)
	const double wx = sensorData.wx;
	const double wy = sensorData.wy;
	const double wz = sensorData.wz;

	const double gyroMagnitude = std::sqrt(wx * wx + wy * wy + wz * wz);

	smoothedAccel += smoothing * (accelMagnitude - smoothedAccel);
	smoothedGyro += smoothing * (gyroMagnitude - smoothedGyro);

	// 3. Calibrate orientation on the first valid packet.
	const double qw = sensorData.qw;
	const double qx = sensorData.qx;
	const double qy = sensorData.qy;
	const double qz = sensorData.qz;

	const double qNorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

	if (qNorm < 1e-6)
		return;

	const double nw = qw / qNorm;
	const double nx = qx / qNorm;
	const double ny = qy / qNorm;
	const double nz = qz / qNorm;

	if (!orientationCalibrated)
	{
		refQw = nw;
		refQx = nx;
		refQy = ny;
		refQz = nz;
		orientationCalibrated = true;
	}

	// Absolute dot product handles q and -q representing
	// the same physical orientation.
	double dot = refQw * nw + refQx * nx + refQy * ny + refQz * nz;

	dot = std::clamp(std::abs(dot), 0.0, 1.0);

	// Relative rotation angle, in degrees.
	const double angleDegrees = 2.0 * std::acos(dot) * 180.0 / 3.14159265358979323846;

	smoothedAngle += smoothing * (angleDegrees - smoothedAngle);

	// Normalize to initial test ranges.
	auto normalizeRiverSignal = [](double value, double noiseFloor, double upperRange)
	{
		double normalized = std::clamp((value - noiseFloor) / (upperRange - noiseFloor), 0.0, 1.0);

		return std::sqrt(normalized); // Makes subtle movements more perceptible.
	};

	// Acceleration controls separation.
	separationControl = normalizeRiverSignal(smoothedAccel, 0.005, 0.12);

	// Angular velocity controls alignment.
	alignmentControl = normalizeRiverSignal(smoothedGyro, 1.0, 30.0);

	// Orientation controls cohesion.
	cohesionControl = normalizeRiverSignal(smoothedAngle, 1.0, 15.0);
}


void ofApp::updateSwarmWeights()
{
	// Keep each flocking force above a nonzero baseline.
	constexpr double minMultiplier = 1.0;
	constexpr double maxMultiplier = 5.0;

	const auto multiplier = [=](double control)
	{
		return minMultiplier + control * (maxMultiplier - minMultiplier);
	};

	swarm.separationWeight = baseSeparationWeight * multiplier(separationControl);

	swarm.alignmentWeight = baseAlignmentWeight * multiplier(alignmentControl);

	swarm.cohesionWeight = baseCohesionWeight * multiplier(cohesionControl);
}


//--------------------------------------------------------------
void ofApp::update()
{
	constexpr double dt = 1.0 / 60.0;

    receivePackets();
	updateSensorControls();
	updateSwarmWeights();

	// Print sensor data once per second for initial testing.
	/*
	if (hasSensorData && simulationTime >= nextControlLogTime)
	{
		ofLogNotice("LPMS Controls")
			<< "separation=" << separationControl
			<< " | alignment=" << alignmentControl
			<< " | cohesion=" << cohesionControl;

		ofLogNotice("LPMS Controls")
			<< "sep=" << separationControl
			<< " ali=" << alignmentControl
			<< " coh=" << cohesionControl
			<< " | weights: sep=" << swarm.separationWeight
			<< " ali=" << swarm.alignmentWeight
			<< " coh=" << swarm.cohesionWeight;

		nextControlLogTime = simulationTime + 1.0;
	}
	*/
	swarm.update(1.0 / 60.0);

	if (simulationTime >= nextOscSendTime)
	{
		sendSwarmOSC();

		nextOscSendTime = simulationTime + oscSendInterval;
	}

	simulationTime += dt;

	/*
	if (simulationTime >= nextSampleTime)
	{
		std::cout << std::fixed << std::setprecision(3)
				  << "t=" << simulationTime
				  << " | center=" << swarm.getCenter().norm()
				  << " | radius=" << swarm.getAverageRadius()
				  << " | avgSpeed=" << swarm.getAverageSpeed()
				  << " | velocityDispersion=" << swarm.getVelocityDispersion()
				  << " | meanVelocity=" << swarm.getMeanVelocity().norm()
				  << '\n';

		const Vec9 meanVelocity = swarm.getMeanVelocity();

		std::cout
			<< " | meanV9=("
			<< meanVelocity[0] << ", "
			<< meanVelocity[1] << ", "
			<< meanVelocity[2] << ", "
			<< meanVelocity[3] << ", "
			<< meanVelocity[4] << ", "
			<< meanVelocity[5] << ", "
			<< meanVelocity[6] << ", "
			<< meanVelocity[7] << ", "
			<< meanVelocity[8] << ")";

		nextSampleTime += 1.0;
	}
	*/
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofEnableDepthTest();

	cam.begin();

	ofSetColor(255);

	// Calculate center in 9D
	Vec9 center = swarm.getCenter();

	for (const auto & boid : swarm.boids) {
		// Position relative to collective center
		Vec9 p = boid.position - center;

		// Project 9D into 3D
		Eigen::Vector3d projected = projection * p;

		ofDrawSphere(
			static_cast<float>(projected.x()),
			static_cast<float>(projected.y()),
			static_cast<float>(projected.z()),
			0.5f);
	}

	cam.end();

	ofDisableDepthTest();

	ofSetColor(255);

	ofSetColor(255);

	std::string info = "9D REYNOLDS SWARM\n"
					   "-------------------------\n"
					   "Boids: "
		+ ofToString(swarm.boids.size()) + "\nCenter distance: "
		+ ofToString(center.norm(), 2) + "\nAverage radius: "
		+ ofToString(swarm.getAverageRadius(), 2) + "\nAverage speed: "
		+ ofToString(swarm.getAverageSpeed(), 3) + "\nVelocity dispersion: "
		+ ofToString(swarm.getVelocityDispersion(), 3) + "\nMean velocity magnitude: "
		+ ofToString(swarm.getMeanVelocity().norm(), 3) + "\nFPS: "
		+ ofToString(ofGetFrameRate(), 1) + "\n\nSPACE: Pause / Resume";

	ofDrawBitmapString(info, 20, 20);
}

void ofApp::sendSwarmOSC()
{
	const std::string baseAddress = "/swarm/" + ofToString(oscSwarmIndex) + "/boids";

	ofxOscMessage positionMsg;

	positionMsg.setAddress(baseAddress + "/position");

	const Vec9 center = swarm.getCenter();

	constexpr double positionScale = 0.5;
	constexpr double velocityScale = 0.5;

	for (const auto & boid : swarm.boids)
	{
		Vec9 relativePosition = boid.position - center;

		for (int d = 0; d < 9; d++)
		{
			positionMsg.addFloatArg(static_cast<float>(relativePosition[d] * positionScale));
		}
	}

	ofxOscMessage velocityMsg;

	velocityMsg.setAddress(baseAddress + "/velocity");

	for (const auto & boid : swarm.boids)
	{
		for (int d = 0; d < 9; d++)
		{
			velocityMsg.addFloatArg(static_cast<float>(boid.velocity[d] * velocityScale));
		}
	}

	oscSender.sendMessage(positionMsg, false);
	oscSender.sendMessage(velocityMsg, false);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
}

//--------------------------------------------------------------
void ofApp::exit()
{
	udp.Close();
}
