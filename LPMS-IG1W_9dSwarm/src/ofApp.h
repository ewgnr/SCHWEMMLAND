#pragma once

#include "Swarm9D.h"
#include "ofxNetwork.h"
#include "ofxOsc.h"
#include "ofMain.h"

#include <cstdint>

#pragma pack(push, 1)
struct ImuPacket
{
	float qw, qx, qy, qz;
	float wx, wy, wz;
	float ax, ay, az;
	uint32_t t;
};
#pragma pack(pop)

static_assert(sizeof(ImuPacket) == 44, "ImuPacket must match the UDP sender");

class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	void exit();

	Swarm9D swarm;
	ofEasyCam cam;

	double simulationTime = 0.0;
	double nextSampleTime = 0.0;

	using ProjectionMatrix = Eigen::Matrix<double, 3, 9>;
	ProjectionMatrix projection = ProjectionMatrix::Zero();

private:
	void receivePackets();

	ofxUDPManager udp;
	ImuPacket sensorData {};
	bool udpInitialized = false;
	bool hasSensorData = false;

	double nextSensorLogTime = 0.0;

	
    // LPMS sensor-control calibration and normalized signals
	bool orientationCalibrated = false;

	// Reference quaternion, stored in (w, x, y, z) order
	double refQw = 1.0;
	double refQx = 0.0;
	double refQy = 0.0;
	double refQz = 0.0;

	double separationControl = 0.0;
	double alignmentControl = 0.0;
	double cohesionControl = 0.0;

	double smoothedAccel = 0.0;
	double smoothedGyro = 0.0;
	double smoothedAngle = 0.0;

	double nextControlLogTime = 0.0;

	void updateSensorControls();

	double baseSeparationWeight = 0.0;
	double baseAlignmentWeight = 0.0;
	double baseCohesionWeight = 0.0;

	void updateSwarmWeights();

	ofxOscSender oscSender;

	std::string oscHost = "127.0.0.1";
	int oscPort = 9006;

	int oscSwarmIndex = 0;

	double nextOscSendTime = 0.0;
	double oscSendInterval = 1.0 / 30.0;

	void sendSwarmOSC();
};
