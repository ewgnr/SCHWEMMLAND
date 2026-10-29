#include "ofApp.h"

#include <cmath>
#include <cstring>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

constexpr double LPMS_TICKS_PER_SECOND = 500.0;

void ofApp::setup() 
{
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(15);

    // Receive the existing LPMS UDP stream.
    udp.Create();
    udp.Bind(9002);
    udp.SetNonBlocking(true);

    camera.setDistance(450);

    ofLogNotice() << "Listening for LPMS data on UDP port 9002";
}

void ofApp::update() 
{
    char buf[1024];

    // Read all currently queued packets, keeping the newest one.
    // This prevents the visualization from gradually lagging behind.
    for (int i = 0; i < 100; ++i) 
    {
        int n = udp.Receive(buf, sizeof(buf));

        if (n <= 0) 
        {
            break;
        }

        if (n != sizeof(ImuPacket)) 
        {
            continue;
        }

        ImuPacket p{};
        std::memcpy(&p, buf, sizeof(p));

        glm::quat q(p.qw, p.qx, p.qy, p.qz);

        // Reject invalid quaternion data.
        float normSquared = glm::dot(q, q);

        if (!std::isfinite(normSquared) || normSquared < 0.000001f) 
        {
            continue;
        }

        sensorOrientation = glm::normalize(q);

        if (hasPreviousTimestamp)
        {
            uint32_t currentTimestamp = p.t;

            uint32_t deltaTicks = currentTimestamp - previousTimestamp;

            deltaSeconds = static_cast<double>(deltaTicks) / LPMS_TICKS_PER_SECOND;
        }
        else
        {
            hasPreviousTimestamp = true;
            deltaSeconds = 0.0;
        }

        previousTimestamp = p.t;

        latestPacket = p;
        hasData = true;
    }
}

void ofApp::draw() 
{
    ofBackground(15);

    ofEnableDepthTest();

    camera.begin();

    // Fixed world coordinate axes.
    //
    // X = red
    // Y = green
    // Z = blue

    const float axisLength = 180.0f;

    ofSetLineWidth(2.0f);

    ofSetColor(255, 70, 70);
    ofDrawLine(0, 0, 0, axisLength, 0, 0);

    ofSetColor(70, 255, 70);
    ofDrawLine(0, 0, 0, 0, axisLength, 0);

    ofSetColor(70, 130, 255);
    ofDrawLine(0, 0, 0, 0, 0, axisLength);

    // Draw the sensor cube.
    ofPushMatrix();

    if (hasData) 
    {
        // Orientation relative to the chosen neutral pose.
        glm::quat relativeOrientation = glm::inverse(referenceOrientation) * sensorOrientation;

        // Apply quaternion rotation to the model.
        ofMultMatrix(glm::mat4_cast(relativeOrientation));
    }

    // Rectangular sensor body.
    ofSetColor(220, 220, 225);
    ofDrawBox(0, 0, 0, 120, 24, 70);

    // Mark one end so its orientation is easier to recognize.
    ofSetColor(255, 160, 40);
    ofDrawBox(45, -14, 0, 20, 5, 50);

    // Local sensor axes, which rotate with the cube.
    ofSetLineWidth(4.0f);

    ofSetColor(255, 70, 70);
    ofDrawLine(0, 0, 0, 85, 0, 0);

    ofSetColor(70, 255, 70);
    ofDrawLine(0, 0, 0, 0, 85, 0);

    ofSetColor(70, 130, 255);
    ofDrawLine(0, 0, 0, 0, 0, 85);

    ofPopMatrix();

    camera.end();

    ofDisableDepthTest();

    // Text overlay.
    ofSetColor(255);

    ofDrawBitmapString("LPMS IG1W - 3D orientation visualizer", 20, 25);

    if (!hasData) 
    {
        ofDrawBitmapString("Waiting for UDP data on port 9001...", 20, 50);
        return;
    }

    const ImuPacket& p = latestPacket;

    ofDrawBitmapString("Quaternion (WXYZ): " + ofToString(p.qw, 4) + "  " + ofToString(p.qx, 4) + "  " + ofToString(p.qy, 4) + "  " + ofToString(p.qz, 4), 20, 50);

    ofDrawBitmapString("Angular velocity: " + ofToString(p.wx, 4) + "  " + ofToString(p.wy, 4) + "  " + ofToString(p.wz, 4), 20, 70);

    ofDrawBitmapString("Linear acceleration: " + ofToString(p.ax, 4) + "  " + ofToString(p.ay, 4) + "  " + ofToString(p.az, 4),20, 90);

    ofDrawBitmapString("Timestamp: " + ofToString(p.t), 20, 110);

    ofDrawBitmapString("Sensor delta: " + ofToString(deltaSeconds * 1000.0, 2) + " ms", 20, 130);

    ofDrawBitmapString("R = reset neutral orientation | Mouse = orbit camera", 20, ofGetHeight() - 20);
}

void ofApp::keyPressed(int key) 
{
    if ((key == 'r' || key == 'R') && hasData) 
    {
        referenceOrientation = sensorOrientation;

        ofLogNotice() << "Neutral orientation reset";
    }
}
