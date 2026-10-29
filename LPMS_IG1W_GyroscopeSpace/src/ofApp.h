
#pragma once

#include "ofMain.h"
#include "ofxNetwork.h"

#include <array>
#include <cstdint>
#include <cstring>
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

static_assert(sizeof(ImuPacket) == 44, "ImuPacket must match the UDP sender");

class ofApp : public ofBaseApp 
{
public:
    void setup();
    void update();
    void draw();
    void exit();

private:
    void receivePackets();

    void drawWaveform(int axis, float zLane,
        const ofColor& color);

    ofxUDPManager udp;
    ofEasyCam camera;

    ImuPacket data{};
    bool hasData = false;
    bool udpInitialized = false;

    std::vector<glm::vec3> gyroHistory;
    static constexpr size_t MAX_HISTORY = 300;

    static constexpr float PLOT_WIDTH = 600.0f;
    static constexpr float PLOT_HEIGHT = 140.0f;
    static constexpr float VALUE_SCALE = 3.5f;
    static constexpr float LANE_SPACING = 180.0f;

    const std::array<ofColor, 3> axisColors = {
        ofColor(255, 75, 75),   // X: red
        ofColor(80, 235, 125),  // Y: green
        ofColor(80, 155, 255)   // Z: blue
    };
};