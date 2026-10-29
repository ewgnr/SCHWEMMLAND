
#include "ofApp.h"

#include <algorithm>
#include <cmath>
#include <cstring>

//--------------------------------------------------------------
void ofApp::setup() 
{
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(14, 17, 23);

    ofEnableSmoothing();

    camera.setDistance(800.0f);
    camera.setNearClip(0.1f);
    camera.setFarClip(5000.0f);
    camera.setTarget(glm::vec3(0.0f));

    // IMPORTANT: Create the socket before binding it.
    udpInitialized = udp.Create();

    if (udpInitialized) 
    {
        udpInitialized = udp.Bind(9005);
    }

    if (udpInitialized) 
    {
        udp.SetNonBlocking(true);

        ofLogNotice("AccelerationSpace") << "Listening for LPMS data on UDP port 9005";
    }
    else 
    {
        ofLogError("AccelerationSpace") << "Could not create or bind UDP socket on port 9005";
    }

    accelerationHistory.reserve(MAX_HISTORY);
    bursts.reserve(MAX_BURSTS);
}

//--------------------------------------------------------------
void ofApp::update() 
{
    receivePackets();
    updateBursts(ofGetLastFrameTime());
}

//--------------------------------------------------------------
void ofApp::receivePackets() 
{
    if (!udpInitialized) 
    {
        return;
    }

    char buffer[512];

    // Drain queued packets so the display stays close to real time.
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

        // These are the linear-acceleration values from your UDP packet.
        currentAcceleration = glm::vec3(data.ax, data.ay, data.az);

        // A gesture impulse is the change between consecutive
        // acceleration samples, not the acceleration magnitude itself.
        latestImpulse = 0.0f;

        if (hasPreviousAcceleration) 
        {
            glm::vec3 difference = currentAcceleration - previousAcceleration;

            latestImpulse = glm::length(difference);

            if (latestImpulse >= IMPULSE_THRESHOLD) 
            {
                ImpulseBurst burst;

                burst.position = currentAcceleration * VECTOR_SCALE;

                float directionLength = glm::length(difference);

                if (directionLength > 0.0001f) 
                {
                    burst.direction = difference / directionLength;
                }

                burst.strength = latestImpulse;
                burst.age = 0.0f;

                bursts.push_back(burst);

                if (bursts.size() > MAX_BURSTS) 
                {
                    bursts.erase(bursts.begin());
                }
            }
        }

        previousAcceleration = currentAcceleration;
        hasPreviousAcceleration = true;

        AccSample sample;
        sample.value = currentAcceleration;
        sample.timestamp = data.t;

        accelerationHistory.push_back(sample);

        if (accelerationHistory.size() > MAX_HISTORY) 
        {
            accelerationHistory.erase(accelerationHistory.begin());
        }
    }
}

//--------------------------------------------------------------
void ofApp::updateBursts(float dt) 
{
    for (auto& burst : bursts) 
    {
        burst.age += dt;
    }

    bursts.erase(std::remove_if(bursts.begin(), bursts.end(), [](const ImpulseBurst& burst) {return burst.age >= BURST_LIFETIME;}), bursts.end());
}

//--------------------------------------------------------------
void ofApp::drawWorldAxes() 
{
    const float axisLength = 260.0f;

    ofSetLineWidth(2.0f);

    // X = red.
    ofSetColor(255, 70, 70);
    ofDrawLine(-axisLength, 0, 0, axisLength, 0, 0);

    // Y = green.
    ofSetColor(70, 255, 110);
    ofDrawLine(0, -axisLength, 0, 0, axisLength, 0);

    // Z = blue.
    ofSetColor(70, 140, 255);
    ofDrawLine(0, 0, -axisLength, 0, 0, axisLength);

    // Subtle reference spheres.
    ofNoFill();

    ofSetColor(65, 75, 90, 100);
    ofDrawSphere(glm::vec3(0.0f), 100.0f);
    ofDrawSphere(glm::vec3(0.0f), 200.0f);
    ofDrawSphere(glm::vec3(0.0f), 300.0f);

    ofFill();
    ofSetLineWidth(1.0f);
}

//--------------------------------------------------------------
void ofApp::drawGestureFossils() 
{
    if (accelerationHistory.size() < 2) 
    {
        return;
    }

    const size_t count = accelerationHistory.size();

    // Draw each historical sample as a point in acceleration space.
    // These points are NOT integrated sensor positions.
    for (size_t i = 1; i < count; ++i) 
    {
        float fade = static_cast<float>(i) / static_cast<float>(count - 1);

        glm::vec3 a = accelerationHistory[i - 1].value * VECTOR_SCALE;

        glm::vec3 b = accelerationHistory[i].value * VECTOR_SCALE;

        // Keep extreme samples within the visual scene.
        float lengthA = glm::length(a);
        float lengthB = glm::length(b);

        if (lengthA > MAX_VECTOR_LENGTH) 
        {
            a *= MAX_VECTOR_LENGTH / lengthA;
        }

        if (lengthB > MAX_VECTOR_LENGTH) 
        {
            b *= MAX_VECTOR_LENGTH / lengthB;
        }

        ofColor trailColor(255, 190, 105, static_cast<unsigned char>(25 + fade * 205));

        ofSetColor(trailColor);
        ofSetLineWidth(1.0f + fade * 3.0f);

        ofDrawLine(a, b);

        if (i % 5 == 0) 
        {
            ofSetColor(255, 220, 155, static_cast<unsigned char>(30 + fade * 170));

            ofDrawSphere(b, 1.0f + fade * 2.0f);
        }
    }

    ofSetLineWidth(1.0f);
}

//--------------------------------------------------------------
void ofApp::drawImpulseBursts() 
{
    for (const auto& burst : bursts) 
    {
        float progress = burst.age / BURST_LIFETIME;
        float remaining = 1.0f - progress;

        float radius = 8.0f + progress * (45.0f + burst.strength * 18.0f);

        radius = std::min(radius, 180.0f);

        unsigned char alpha = static_cast<unsigned char>(remaining * 190.0f);

        // Build a ring perpendicular to the impulse direction.
        glm::vec3 normal = burst.direction;

        glm::vec3 helper = std::abs(normal.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);

        glm::vec3 tangent = glm::normalize(glm::cross(normal, helper));

        glm::vec3 bitangent = glm::normalize(glm::cross(normal, tangent));

        const int segments = 48;
        const float twoPi = glm::two_pi<float>();

        ofSetColor(255, 145, 55, alpha);
        ofSetLineWidth(1.0f + remaining * 2.0f);

        for (int i = 0; i < segments; ++i) 
        {
            float angleA = twoPi * static_cast<float>(i) / segments;

            float angleB = twoPi * static_cast<float>(i + 1) / segments;

            glm::vec3 pointA = burst.position + radius * (std::cos(angleA) * tangent + std::sin(angleA) * bitangent);

            glm::vec3 pointB = burst.position + radius * (std::cos(angleB) * tangent + std::sin(angleB) * bitangent);

            ofDrawLine(pointA, pointB);
        }
    }

    ofSetLineWidth(1.0f);
}

//--------------------------------------------------------------
void ofApp::drawLiveVector() 
{
    if (!hasData) 
    {
        return;
    }

    glm::vec3 endpoint = currentAcceleration * VECTOR_SCALE;

    float length = glm::length(endpoint);

    if (length > MAX_VECTOR_LENGTH) 
    {
        endpoint *= MAX_VECTOR_LENGTH / length;
        length = MAX_VECTOR_LENGTH;
    }

    if (length < 0.001f) 
    {
        ofSetColor(255, 245, 215);
        ofDrawSphere(glm::vec3(0.0f), 5.0f);
        return;
    }

    // Colored projections show each acceleration component.
    ofSetLineWidth(2.0f);

    ofSetColor(255, 70, 70, 190);
    ofDrawLine(glm::vec3(0.0f), glm::vec3(endpoint.x, 0.0f, 0.0f));

    ofSetColor(70, 255, 110, 190);
    ofDrawLine(glm::vec3(0.0f), glm::vec3(0.0f, endpoint.y, 0.0f));

    ofSetColor(70, 140, 255, 190);
    ofDrawLine(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, endpoint.z));

    // Main acceleration vector.
    ofSetColor(255, 245, 215);
    ofSetLineWidth(4.0f);

    ofDrawArrow(glm::vec3(0.0f), endpoint, 10.0f);

    ofSetColor(255);
    ofDrawSphere(endpoint, 5.0f);

    ofSetLineWidth(1.0f);
}

//--------------------------------------------------------------
void ofApp::drawHud() 
{
    ofSetColor(245);
    ofDrawBitmapString("ACCELERATION SPACE", 20, 28);

    ofSetColor(170);
    ofDrawBitmapString("Linear acceleration | 3D gesture fossils + impulse bursts", 20, 48);

    ofDrawBitmapString("Mouse: orbit / zoom    R: clear history and bursts", 20, 68);

    if (!udpInitialized) 
    {
        ofSetColor(255, 90, 90);
        ofDrawBitmapString("UDP ERROR: Could not create or bind port 9001", 20, 100);
        return;
    }

    if (!hasData) 
    {
        ofSetColor(255, 190, 80);
        ofDrawBitmapString("Waiting for LPMS packets on UDP port 9001...", 20, 100);
        return;
    }

    ofSetColor(255, 70, 70);
    ofDrawBitmapString("X: " + ofToString(data.ax, 4), 20, 105);

    ofSetColor(70, 255, 110);
    ofDrawBitmapString("Y: " + ofToString(data.ay, 4), 20, 125); 

    ofSetColor(70, 140, 255);
    ofDrawBitmapString("Z: " + ofToString(data.az, 4), 20, 145);

    float magnitude = glm::length(currentAcceleration);

    ofSetColor(220);
    ofDrawBitmapString("Magnitude: " + ofToString(magnitude, 4), 20, 170);

    ofDrawBitmapString("Sample change: " + ofToString(latestImpulse, 4), 20, 190);

    ofDrawBitmapString("History: " + ofToString(accelerationHistory.size()) + " / " + ofToString(MAX_HISTORY), 20, 210);

    ofDrawBitmapString("Active impulse bursts: " + ofToString(bursts.size()), 20, 230);

    ofDrawBitmapString("Timestamp: " + ofToString(data.t), 20, 250);

    ofSetColor(135);
    ofDrawBitmapString("Trail shows acceleration-space history, not physical position.", 20, ofGetHeight() - 20);
}

//--------------------------------------------------------------
void ofApp::draw() 
{
    ofBackground(14, 17, 23);

    ofEnableDepthTest();
    ofEnableAlphaBlending();

    camera.begin();

    drawWorldAxes();
    drawGestureFossils();
    drawImpulseBursts();
    drawLiveVector();

    camera.end();

    ofDisableDepthTest();

    drawHud();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) 
{
    if (key == 'r' || key == 'R') 
    {
        accelerationHistory.clear();
        bursts.clear();

        latestImpulse = 0.0f;
        hasPreviousAcceleration = false;

        ofLogNotice("AccelerationSpace") << "Acceleration visualization history cleared";
    }
}

//--------------------------------------------------------------
void ofApp::exit() 
{
    if (udpInitialized) 
    {
        udp.Close();
    }
}
