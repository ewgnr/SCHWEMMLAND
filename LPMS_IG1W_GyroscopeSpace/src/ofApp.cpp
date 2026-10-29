
#include "ofApp.h"

#include <algorithm>
#include <cmath>

//--------------------------------------------------------------
void ofApp::setup() 
{
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(14, 17, 23);

    camera.setDistance(850.0f);
    camera.setNearClip(0.1f);
    camera.setFarClip(5000.0f);
    camera.setTarget(glm::vec3(0.0f));

    udpInitialized = udp.Create();

    if (udpInitialized) 
    {
        udpInitialized = udp.Bind(9004);
    }

    if (udpInitialized) 
    {
        udp.SetNonBlocking(true);

        ofLogNotice("GyroscopeSpace") << "Listening on UDP port 9004";
    }
    else 
    {
        ofLogError("GyroscopeSpace") << "Could not create or bind UDP socket on port 9004";
    }

    gyroHistory.reserve(MAX_HISTORY);
}

//--------------------------------------------------------------
void ofApp::update() 
{
    receivePackets();
}

//--------------------------------------------------------------
void ofApp::receivePackets() 
{
    if (!udpInitialized) 
    {
        return;
    }

    char buffer[512];

    // Drain queued packets and keep the latest sample.
    for (int i = 0; i < 100; ++i) 
    {
        int bytes = udp.Receive(buffer, sizeof(buffer));

        if (bytes <= 0) 
        {
            break;
        }

        if (bytes != static_cast<int>(sizeof(ImuPacket))) 
        {
            continue;
        }

        ImuPacket incoming{};
        std::memcpy(&incoming, buffer, sizeof(incoming));

        const float values[] = {
            incoming.qw, incoming.qx, incoming.qy, incoming.qz,
            incoming.wx, incoming.wy, incoming.wz,
            incoming.ax, incoming.ay, incoming.az
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
        {
            continue;
        }

        data = incoming;
        hasData = true;

        gyroHistory.emplace_back(data.wx, data.wy, data.wz);

        if (gyroHistory.size() > MAX_HISTORY) 
        {
            gyroHistory.erase(gyroHistory.begin());
        }
    }
}

//--------------------------------------------------------------
void ofApp::drawWaveform(int axis, float zLane, const ofColor& color) 
{
    if (gyroHistory.empty()) 
    {
        return;
    }

    const float step = PLOT_WIDTH / static_cast<float>(MAX_HISTORY - 1);

    // Zero baseline.
    ofSetColor(65, 72, 85, 180);
    ofSetLineWidth(1.0f);

    ofDrawLine(-PLOT_WIDTH * 0.5f, 0.0f, zLane, PLOT_WIDTH * 0.5f, 0.0f, zLane);

    // Vertical time guides.
    for (int i = 0; i <= 10; ++i) 
    {
        float x = -PLOT_WIDTH * 0.5f + PLOT_WIDTH * i / 10.0f;

        ofSetColor(55, 62, 75, 100);

        ofDrawLine(x, -PLOT_HEIGHT, zLane, x, PLOT_HEIGHT, zLane);
    }

    // Waveform.
    ofSetColor(color);
    ofSetLineWidth(2.5f);

    for (size_t i = 1; i < gyroHistory.size(); ++i) 
    {
        float x1 = -PLOT_WIDTH * 0.5f + static_cast<float>(i - 1) * step;

        float x2 = -PLOT_WIDTH * 0.5f + static_cast<float>(i) * step;

        float v1 = std::clamp(gyroHistory[i - 1][axis], -400.0f, 400.0f);

        float v2 = std::clamp(gyroHistory[i][axis], -400.0f, 400.0f);

        float y1 = v1 * VALUE_SCALE;
        float y2 = v2 * VALUE_SCALE;

        ofDrawLine(x1, y1, zLane, x2, y2, zLane);
    }

    // Bright marker at the newest sample.
    const auto& latest = gyroHistory.back();

    float latestX = -PLOT_WIDTH * 0.5f + static_cast<float>(gyroHistory.size() - 1) * step;

    float latestY = std::clamp(latest[axis], -400.0f, 400.0f) * VALUE_SCALE;

    ofSetColor(255);
    ofDrawSphere(latestX, latestY, zLane, 4.0f);

    ofSetLineWidth(1.0f);
}

//--------------------------------------------------------------
void ofApp::draw() 
{
    // ================= 3D SCENE =================
    camera.begin();

    // Reference axes.
    ofSetLineWidth(2.0f);

    ofSetColor(255, 75, 75);
    ofDrawLine(-100, 0, 0, 100, 0, 0);

    ofSetColor(80, 235, 125);
    ofDrawLine(0, -100, 0, 0, 100, 0);

    ofSetColor(80, 155, 255);
    ofDrawLine(0, 0, -100, 0, 0, 100);

    // Three separate waveform planes.
    const float lanes[3] = {-LANE_SPACING, 0.0f, LANE_SPACING};

    for (int axis = 0; axis < 3; ++axis) 
    {
        drawWaveform(axis, lanes[axis], axisColors[axis]);
    }

    // Live angular-velocity vector.
    if (hasData) 
    {
        glm::vec3 gyroVector(data.wx, data.wy, data.wz);

        gyroVector *= VALUE_SCALE;

        if (glm::length(gyroVector) > 0.01f) 
        {
            ofSetColor(255, 245, 220);
            ofSetLineWidth(4.0f);

            ofDrawArrow(glm::vec3(0.0f), gyroVector, 9.0f);

            ofSetLineWidth(1.0f);
        }
    }

    camera.end();

    // ================= 2D HUD =================
    ofSetColor(245);
    ofDrawBitmapString("GYROSCOPE SPACE", 20, 28);

    ofSetColor(170);
    ofDrawBitmapString("Angular velocity | deg/s", 20, 48);

    ofDrawBitmapString("Drag to orbit camera | Scroll to zoom", 20, 68);

    if (!udpInitialized) 
    {
        ofSetColor(255, 90, 90);
        ofDrawBitmapString("UDP ERROR: Could not bind port 9001", 20, 100);
        return;
    }

    if (!hasData) 
    {
        ofSetColor(255, 190, 80);
        ofDrawBitmapString("Waiting for IMU packets on port 9001...", 20, 100);
        return;
    }

    ofSetColor(axisColors[0]);
    ofDrawBitmapString("X: " + ofToString(data.wx, 2) + " deg/s", 20, 105);

    ofSetColor(axisColors[1]);
    ofDrawBitmapString("Y: " + ofToString(data.wy, 2) + " deg/s", 20, 125);

    ofSetColor(axisColors[2]);
    ofDrawBitmapString("Z: " + ofToString(data.wz, 2) + " deg/s", 20, 145);

    ofSetColor(170);
    ofDrawBitmapString("History: " + ofToString(gyroHistory.size()) + " / " + ofToString(MAX_HISTORY), 20, 170);

    float speed = std::sqrt(data.wx * data.wx + data.wy * data.wy + data.wz * data.wz);

    ofDrawBitmapString("Combined speed: " + ofToString(speed, 2) + " deg/s", 20, 190);
}

//--------------------------------------------------------------
void ofApp::exit() 
{
    if (udpInitialized) 
    {
        udp.Close();
    }
}
