
#pragma once

#include "ofMain.h"
#include "ofxNetwork.h"

#include <cstdint>
#include <vector>

#pragma pack(push, 1)
struct ImuPacket 
{
    float qw, qx, qy, qz;
    float wx, wy, wz;
    float ax, ay, az;
    uint32_t t;
};
#pragma pack(pop)

static_assert(sizeof(ImuPacket) == 44,
    "ImuPacket must match the UDP sender");

class ofApp : public ofBaseApp {
public:
    void setup();
    void update();
    void draw();
    void exit();

private:
    void receivePackets();

    ofxUDPManager udp;

    ImuPacket data{};
    bool hasData = false;

    // Derived Euler angles, in degrees.
    // These are diagnostic values derived from the quaternion.
    glm::vec3 eulerDegrees{ 0.0f };

    // Recent yaw samples, in degrees, for the polar trail.
    std::vector<float> yawHistory;
    static constexpr size_t MAX_HISTORY = 300;

    bool udpInitialized = false;

    glm::quat sensorOrientation{ 1, 0, 0, 0 };
    glm::quat referenceOrientation{ 1, 0, 0, 0 };

    std::vector<glm::vec3> directionHistory;
    static constexpr size_t MAX_DIRECTION_HISTORY = 250;

    void keyPressed(int key);

};
