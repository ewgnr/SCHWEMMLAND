// namespace Space    #pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "Boid.h"
#include "BoidDefs.h"

class SwarmOSCReceiver
{
public:
	SwarmOSCReceiver() = default;

	void setup(int port, int mB)
	{
		receiver.setup(port);
		
		maxBoids = mB;
		buffers[0].resize(maxBoids);
		buffers[1].resize(maxBoids);
		
		ofLogNotice() << "SwarmOSCReceiver listening on port " << port;
	}

	void update()
	{
		while (receiver.hasWaitingMessages())
		{
			ofxOscMessage msg;
			receiver.getNextMessage(msg);
			handleMessage(msg);
		}

		// --------------------------------------------------
		// ONLY SWAP WHEN FULL FRAME IS COMPLETE
		// --------------------------------------------------
		if (gotPosition && gotVelocity)
		{
			int w = writeIndex.load(std::memory_order_relaxed);
			int r = 1 - w;

			readIndex.store(w, std::memory_order_release);
			writeIndex.store(r, std::memory_order_relaxed);

			gotPosition = false;
			gotVelocity = false;
		}
	}
	
	const std::vector<Boid>& getBoids() const
	{
		int r = readIndex.load(std::memory_order_acquire);
		return buffers[r];
	}
	
	int getSwarmIndex() const { return swarmIndex; }

	std::vector<Boid>& getWriteBuffer()
	{
		int w = writeIndex.load(std::memory_order_relaxed);
		return buffers[w];
	}

	void drawDebug(float startX = 20.0f, float startY = 20.0f)
	{
		ofBackground(0);

		const auto& boids = getBoids();

		int y = startY;

		ofDrawBitmapStringHighlight("Swarm " + ofToString(swarmIndex) + " | Boids: " + ofToString(boids.size()), startX, y);

		y += 25;

		for (const auto& b : boids)
		{
			ofSetColor(255);

			std::string pos = "Boid " + ofToString(b.index) + "  POS  "
				"D1("
				+ ofToString(b.position[0], 2) + ","
				+ ofToString(b.position[1], 2) + ","
				+ ofToString(b.position[2], 2) + ") "
				"D2("
				+ ofToString(b.position[3], 2) + ","
				+ ofToString(b.position[4], 2) + ","
				+ ofToString(b.position[5], 2) + ") "
				"D3("
				+ ofToString(b.position[6], 2) + ","
				+ ofToString(b.position[7], 2) + ","
				+ ofToString(b.position[8], 2) + ")";

			ofDrawBitmapString(pos, startX, y);

			y += 15;

			ofSetColor(180, 180, 255);

			std::string vel = "        VEL  "
				"D1("
				+ ofToString(b.velocity[0], 2) + ","
				+ ofToString(b.velocity[1], 2) + ","
				+ ofToString(b.velocity[2], 2) + ") "
				"D2("
				+ ofToString(b.velocity[3], 2) + ","
				+ ofToString(b.velocity[4], 2) + ","
				+ ofToString(b.velocity[5], 2) + ") "
				"D3("
				+ ofToString(b.velocity[6], 2) + ","
				+ ofToString(b.velocity[7], 2) + ","
				+ ofToString(b.velocity[8], 2) + ")";

			ofDrawBitmapString(vel, startX, y);

			y += 22;
		}

		ofSetColor(255);
	}

	void printMinMax()
	{
		const auto& boids = getBoids();
		if (boids.empty()) return;

		float posMin[3][3], posMax[3][3];
		float velMin[3][3], velMax[3][3];

		for (int d = 0; d < 3; d++)
		{
			for (int axis = 0; axis < 3; axis++)
			{
				posMin[d][axis] = posMax[d][axis] = boids[0].position[d * 3 + axis];
				velMin[d][axis] = velMax[d][axis] = boids[0].velocity[d * 3 + axis];
			}
		}

		for (const auto& b : boids)
		{
			for (int d = 0; d < 3; d++) 
			{
				for (int axis = 0; axis < 3; axis++) 
				{
					float p = b.position[d * 3 + axis];
					if (p < posMin[d][axis]) posMin[d][axis] = p;
					if (p > posMax[d][axis]) posMax[d][axis] = p;

					float v = b.velocity[d * 3 + axis];
					if (v < velMin[d][axis]) velMin[d][axis] = v;
					if (v > velMax[d][axis]) velMax[d][axis] = v;
				}
			}
		}

		std::cout << "Position Min/Max per Axis:\n";
		for (int d = 0; d < 3; d++) 
		{
			std::cout << "D" << (d + 1)
				<< " x:[" << posMin[d][0] << "," << posMax[d][0] << "] "
				<< "y:[" << posMin[d][1] << "," << posMax[d][1] << "] "
				<< "z:[" << posMin[d][2] << "," << posMax[d][2] << "]\n";
		}

		std::cout << "Velocity Min/Max per Axis:\n";
		for (int d = 0; d < 3; d++) 
		{
			std::cout << "D" << (d + 1)
				<< " x:[" << velMin[d][0] << "," << velMax[d][0] << "] "
				<< "y:[" << velMin[d][1] << "," << velMax[d][1] << "] "
				<< "z:[" << velMin[d][2] << "," << velMax[d][2] << "]\n";
		}
	}

	int getTimelineYear() const
	{
		return timelineYear;
	}

private:
	ofxOscReceiver receiver;
	int swarmIndex = -1;
	int timelineYear = 1450;
	int maxBoids = 0;

	std::vector<Boid> buffers[2];
	std::atomic<int> writeIndex{ 0 };
	std::atomic<int> readIndex{ 1 };

	bool gotPosition = false;
	bool gotVelocity = false;

	bool frameReady = false;

	void handleMessage(const ofxOscMessage& msg)
	{
		if (msg.getAddress() == "/timeline/year")
		{
			timelineYear = msg.getArgAsInt(0);
			return;
		}

		auto addr = ofSplitString(msg.getAddress(), "/");
		if (addr.size() != 5 || addr[1] != "swarm") return;

		swarmIndex = ofToInt(addr[2]);
		std::string type = addr[4];

		int totalFloats = msg.getNumArgs();
		if (totalFloats % DimsPerBoid != 0) return;

		int numBoids = static_cast<int>(totalFloats / DimsPerBoid);
		numBoids = std::min(numBoids, maxBoids);

		// WRITE BUFFER
		int w = writeIndex.load(std::memory_order_relaxed);
		auto& target = buffers[w];

		int argIndex = 0;

		// --------------------------------------------------
		// WRITE DATA INTO BACK BUFFER
		// --------------------------------------------------
		for (int i = 0; i < numBoids; i++)
		{
			for (int d = 0; d < DimsPerBoid; d++)
			{
				float v = msg.getArgAsFloat(argIndex++);

				// safety clamp (prevents NaNs -> audio clicks)
				if (!std::isfinite(v)) v = 0.0f;

				if (type == "position")
					target[i].position[d] = v;
				else if (type == "velocity")
					target[i].velocity[d] = v;
			}
		}

		// --------------------------------------------------
		// CLEAR UNUSED BOIDS
		// --------------------------------------------------
		for (int i = numBoids; i < maxBoids; i++)
		{
			for (int d = 0; d < DimsPerBoid; d++)
			{
				if (type == "position")
					target[i].position[d] = 0.0f;
				else if (type == "velocity")
					target[i].velocity[d] = 0.0f;
			}
		}

		// --------------------------------------------------
		// MARK WHICH HALF OF FRAME ARRIVED
		// --------------------------------------------------
		if (type == "position") gotPosition = true;
		else if (type == "velocity") gotVelocity = true;
	}
};