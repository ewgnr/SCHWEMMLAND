
#include "ofApp.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

void ofApp::setup() 
{
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(14, 17, 23);

    udp.Create();
    udpInitialized = udp.Bind(9003);
    udp.SetNonBlocking(true);

    if (udpInitialized) 
    {
        ofLogNotice("OrientationSpace") << "Listening for LPMS UDP on port 9003";
    }
    else 
    {
        ofLogError("OrientationSpace") << "Could not bind UDP port 9003";
    }
}

void ofApp::exit() 
{
    udp.Close();
}

void ofApp::receivePackets() 
{
    if (!udpInitialized) return;

    char buffer[1024];

    // Drain queued packets so we use the newest available data.
    for (int i = 0; i < 100; ++i) 
    {
        int n = udp.Receive(buffer, sizeof(buffer));

        if (n <= 0) break;
        if (n != static_cast<int>(sizeof(ImuPacket))) continue;

        ImuPacket packet{};
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

        if (!valid) continue;

        // Validate quaternion before accepting it.
        glm::quat q(packet.qw, packet.qx, packet.qy, packet.qz);

        const float normSquared = glm::dot(q, q);

        if (!std::isfinite(normSquared) || normSquared < 1e-6f) {
            continue;
        }

        data = packet;
        hasData = true;
    }
}


void ofApp::update() 
{
    receivePackets();
    if (!hasData) return;

    glm::quat q(data.qw, data.qx, data.qy, data.qz);
    float n2 = glm::dot(q, q);

    if (!std::isfinite(n2) || n2 < 1e-6f) return;

    sensorOrientation = glm::normalize(q);

    // Transform the sensor's local forward axis into world space.
    glm::vec3 forward = sensorOrientation * glm::vec3(0, 0, -1);

    directionHistory.push_back(forward);

    if (directionHistory.size() > MAX_DIRECTION_HISTORY) 
    {
        directionHistory.erase(directionHistory.begin());
    }

    glm::vec3 euler = glm::degrees(glm::eulerAngles(sensorOrientation));
    eulerDegrees = euler;
}

void ofApp::keyPressed(int key) 
{
    if (key == 'r' || key == 'R') 
    {
        referenceOrientation = sensorOrientation;
        directionHistory.clear();
    }
}

void ofApp::draw() 
{
    ofBackground(14, 17, 23);

    ofSetColor(240);
    ofDrawBitmapString("ORIENTATION SPACE / 3D FRAME", 20, 25);
    ofSetColor(150);
    ofDrawBitmapString("R: set current pose as reference", 20, 45);

    if (!hasData) 
    {
        ofSetColor(230);
        ofDrawBitmapString("Waiting for LPMS data on UDP 9001...", 20, 75);
        return;
    }

    // Use a fixed camera view for comparing world and sensor axes.
    ofPushMatrix();

    ofTranslate(ofGetWidth() * 0.5f, ofGetHeight() * 0.52f);
    ofScale(1, -1, 1);

    const float axisLength = 150.0f;

    // Fixed world coordinate frame.
    ofSetLineWidth(2.0f);
    ofSetColor(255, 85, 90);
    ofDrawLine(0, 0, 0, axisLength, 0, 0);

    ofSetColor(90, 230, 145);
    ofDrawLine(0, 0, 0, 0, axisLength, 0);

    ofSetColor(95, 160, 255);
    ofDrawLine(0, 0, 0, 0, 0, axisLength);

    // Draw the sensor frame after applying its orientation.
    glm::quat relative = glm::normalize(glm::inverse(referenceOrientation) * sensorOrientation);

    ofPushMatrix();
    ofMultMatrix(glm::mat4_cast(relative));

    ofSetColor(255, 85, 90);
    ofDrawLine(0, 0, 0, axisLength, 0, 0);

    ofSetColor(90, 230, 145);
    ofDrawLine(0, 0, 0, 0, axisLength, 0);

    ofSetColor(95, 160, 255);
    ofDrawLine(0, 0, 0, 0, 0, axisLength);

    // A small wireframe cube makes the sensor frame easier to read.
    ofNoFill();
    ofSetColor(230, 230, 240);
    ofDrawBox(0, 0, 0, 45);
    ofFill();

    ofPopMatrix();

    // Trail of the sensor's forward direction.
    // This is a 3D trajectory projected onto the current view plane.
    for (size_t i = 1; i < directionHistory.size(); ++i) 
    {
        float fade = float(i) / directionHistory.size();

        glm::vec3 a = directionHistory[i - 1] * 100.0f;
        glm::vec3 b = directionHistory[i] * 100.0f;

        ofSetColor(95, 160, 255, int(fade * 180));
        ofDrawLine(a.x, a.y, a.z, b.x, b.y, b.z);
    }

    ofSetLineWidth(1.0f);
    ofPopMatrix();

    // Telemetry panel.
    ofSetColor(255, 100, 105);
    ofDrawBitmapString("Q: " + ofToString(data.qw, 3) + ", " + ofToString(data.qx, 3), 20, ofGetHeight() - 100);

    ofSetColor(100, 230, 150);
    ofDrawBitmapString("   " + ofToString(data.qy, 3) + ", " + ofToString(data.qz, 3), 20, ofGetHeight() - 80);

    ofSetColor(150, 190, 255);
    ofDrawBitmapString("Euler XYZ: " + ofToString(eulerDegrees.x, 1) + ", " + ofToString(eulerDegrees.y, 1) + ", " + ofToString(eulerDegrees.z, 1) + " deg", 20, ofGetHeight() - 45);
}

