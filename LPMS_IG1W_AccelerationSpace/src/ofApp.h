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

static_assert(sizeof(ImuPacket) == 44, "ImuPacket must match the UDP sender");

class ofApp : public ofBaseApp 
{
public:
    void setup();
    void update();
    void draw();
    void exit();
    void keyPressed(int key);

private:
    struct AccSample 
    {
        glm::vec3 value{ 0.0f };
        uint32_t timestamp = 0;
    };

    struct ImpulseBurst 
    {
        glm::vec3 position{ 0.0f };
        glm::vec3 direction{ 0.0f, 0.0f, 1.0f };
        float strength = 0.0f;
        float age = 0.0f;
    };

    void receivePackets();
    void updateBursts(float dt);
    void drawWorldAxes();
    void drawGestureFossils();
    void drawImpulseBursts();
    void drawLiveVector();
    void drawHud();

    ofxUDPManager udp;
    ofEasyCam camera;

    ImuPacket data{};

    bool udpInitialized = false;
    bool hasData = false;
    bool hasPreviousAcceleration = false;

    glm::vec3 currentAcceleration{ 0.0f };
    glm::vec3 previousAcceleration{ 0.0f };

    std::vector<AccSample> accelerationHistory;
    std::vector<ImpulseBurst> bursts;

    static constexpr size_t MAX_HISTORY = 300;
    static constexpr size_t MAX_BURSTS = 32;

    // Visualization parameters, not sensor calibration values.
    static constexpr float VECTOR_SCALE = 1500.0f;
    static constexpr float MAX_VECTOR_LENGTH = 180.0f;

    static constexpr float IMPULSE_THRESHOLD = 0.045f;
    static constexpr float BURST_LIFETIME = 1.5f;

    float latestImpulse = 0.0f;
};