
#pragma once

#include "ofMain.h"
#include "ofxNetwork.h"

#include <cstdint>

struct ImuPacket 
{
    float qw, qx, qy, qz;
    float wx, wy, wz;
    float ax, ay, az;
    uint32_t t;
};

static_assert(sizeof(ImuPacket) == 44,
    "Unexpected IMU packet size");

class ofApp : public ofBaseApp 
{
public:
    void setup() override;
    void update() override;
    void draw() override;
    void keyPressed(int key) override;

private:
    ofxUDPManager udp;

    ImuPacket latestPacket{};

    bool hasData = false;

    // Latest orientation received from the LPMS.
    glm::quat sensorOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    // Reference orientation established by pressing R.
    glm::quat referenceOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    ofEasyCam camera;

    uint32_t previousTimestamp = 0;
    bool hasPreviousTimestamp = false;

    double deltaSeconds = 0.0;
};
