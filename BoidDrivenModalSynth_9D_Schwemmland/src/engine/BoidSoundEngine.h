#pragma once

#include "ofMath.h"
#include "ofSoundStream.h"
#include "ModalBank2D.h"
#include "AmbiEncode2DThirdOrder.h"
#include "AmbiDecode2DThirdOrder.h"
#include "AmbisonicParams.h"
#include "BoidSpatialSystem.h"
#include "TriggerEngine.h"
#include "ControlMapping.h"
#include "ModalParams.h"
#include "ModalSynthesis.h"
#include "AmbisonicRenderer.h"
#include "VbapRenderer.h"
#include "OutputStage.h"
#include "ParamRegistry.h"
#include "BoidDefs.h"

#include <cstdlib>
#include <array>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <atomic>

constexpr std::size_t NUM_SPEAKERS = 4;
constexpr double SAMPLERATE = 48000.0;  
constexpr std::size_t BUFFERSIZE = 1024;

enum class SpatialRendererType
{
    Ambisonics,
    VBAP
};

// ============================================================
// ENGINE
// ============================================================
class AdaptiveBoidSoundEngine : public ofBaseSoundOutput
{
public:

    // ========================================================
    // SETUP
    // ========================================================
    void setup(int maxBoids)
    {
        Nmax = maxBoids;

        spatialSystem.setup(Nmax);
        triggerEngine.setup(Nmax, SAMPLERATE);
        phase.assign(Nmax, 0.0);
        control.setup(Nmax);
        modalSynth.setup(Nmax, SAMPLERATE);
        spatialParams.resize(maxBoids);
        ambi.setup(NUM_SPEAKERS, Nmax);
        vbap.setup(NUM_SPEAKERS, Nmax);
        vbap.setSmoothing(SAMPLERATE, 10.0);
        outputStage.setup({ 0, 1, 2, 3 }, { 0, 1, 2, 3 }); // speakers & output channels

        speakerBuffer.resize(NUM_SPEAKERS);
    }


    // ========================================================
    // SPATIAL FEATURES
    // ========================================================
    void updateBoids(const std::vector<Boid>& boidsIn)
    {
        // store pointer (zero-copy)
        boidsPtr = &boidsIn;

        if (boidsIn.empty())
            return;

        // use directly (no indirection needed)
        spatialSystem.update(boidsIn);

        const MultiSpatialFrame* spatial = spatialSystem.getFrame();

        control.update(spatial->sub[0], spatial->sub[1], spatial->sub[2], static_cast<int>(boidsIn.size()));
    }


    // ========================================================
    // AUDIO OUTPUT
    // ========================================================
    void audioOut(ofSoundBuffer& buffer) override
    {
        // ----------------------------------------
        // 1. Get CONTROL FRAME (single snapshot)
        // ----------------------------------------
        const ControlFrame* frame = control.getFrame();

        const double dt = static_cast<double>(buffer.getNumFrames()) / SAMPLERATE;
        
        if (!frame || frame->N <= 0)
        {
            // hard silence (audio-safe)
            const int total = buffer.getNumFrames() * buffer.getNumChannels();
            for (int i = 0; i < total; ++i)
                buffer[i] = 0.0f;
            return;
        }
        
        const int N_local = frame->N;

        // ----------------------------------------
        // 2. Resize phase safely
        // ----------------------------------------
        // if (phase.size() < N_local)
        //    phase.resize(N_local, 0.0);

        // ----------------------------------------
        // 3. Apply modal parameters (same frame)
        // ----------------------------------------
        modalSynth.applyParams(&frame->buffer, N_local);

        // ----------------------------------------
        // 4. Trigger system (same frame)
        // ----------------------------------------
        triggerEngine.computeRate(frame->driveField, N_local, dt);
        const std::vector<double>& rate = triggerEngine.getRate();

        // ----------------------------------------
        // 5. Spatial frame (still separate system)
        // ----------------------------------------
        const MultiSpatialFrame* spatial = spatialSystem.getFrame();
        
        if (!spatial || spatial->sub[0].wrappedPos.empty())
        {
            const int total = buffer.getNumFrames() * buffer.getNumChannels();
            for (int i = 0; i < total; ++i)
                buffer[i] = 0.0f;
            return;
        }
        
        // ----------------------------------------
        // 5.5 Resolve cross-system size mismatch
        // ----------------------------------------
        const int N_spatial = spatial->sub[0].N;
        const int N_render = std::min(N_local, N_spatial);
        
        // ----------------------------------------
        // 6. Render
        // ----------------------------------------
        renderAudio(buffer, *spatial, rate, N_render);
    }
    

    const std::vector<Boid>& getBoids() const
    {
        static const std::vector<Boid> empty;
        return boidsPtr ? *boidsPtr : empty;
    }


    glm::vec3 getBoidPosition(int i) const
    {
        const MultiSpatialFrame* spatial = spatialSystem.getFrame();
        return spatial->sub[0].normalizedPos[i];
    }


    void setRenderer(SpatialRendererType type)
    {
        rendererType = type;
    }


private:

    // ========================================================
    // MODAL SYNTHESIS & AMBISONICS
    // ========================================================
    void renderAudio(ofSoundBuffer& buffer,
        const MultiSpatialFrame& spatial,
        const std::vector<double>& rate,
        int N)
    {
    
        /*
         *
         * The complete audio path is retained here:
         *
         * clear
         * -> trigger accumulator
         * -> excitation
         * -> modal render
         * -> ambisonic encoding
         * -> ambisonic accumulation
         * -> decoder
         * -> output drive
         * -> output routing
         */
        
        const SpatialFrame& geometry = spatial.sub[0];

        const int CH = buffer.getNumChannels();
        const int frames = buffer.getNumFrames();

        const double width = SPATIAL_WIDTH.load(std::memory_order_relaxed);
        const double gainExp = SPATIAL_GAIN_EXP.load(std::memory_order_relaxed);
        const double distanceMin = SPATIAL_DISTANCE_MIN.load(std::memory_order_relaxed);
        const double distanceMax = SPATIAL_DISTANCE_MAX.load(std::memory_order_relaxed);
        const double drive = OUTPUT_TANH_DRIVE.load(std::memory_order_relaxed);
        const double master = OUTPUT_MASTER_GAIN.load(std::memory_order_relaxed);
        
        const double spread = std::clamp(width, 0.2, 3.0);
        const int spreadSteps = static_cast<int>(2 + spread * 3.0);

        const double spreadWidth = 0.25 * spread;

        for (int j = 0; j < N; ++j)
        {
            const glm::dvec3& pos = geometry.continuousPos[j];

            const double az = Space::azimuth(pos);

            double dist = Space::radialNorm(pos);
            dist = std::tanh(dist * 0.5);
            dist = std::clamp(dist, distanceMin, distanceMax);

            spatialParams[j].az = az;
            spatialParams[j].dist = dist;
            spatialParams[j].distGain = std::exp(-gainExp * dist);
            spatialParams[j].spread = spreadWidth;
            spatialParams[j].spreadSteps = spreadSteps;
        }

        switch (rendererType)
        {
        case SpatialRendererType::Ambisonics:
            ambi.prepareParams(spatialParams, N);
            break;

        case SpatialRendererType::VBAP:
            vbap.prepareParams(spatialParams, N);
            break;
        }

        for (std::size_t i = 0; i < frames; i++)
        {
            // ------------------------------------------------
            // Output Clear
            // ------------------------------------------------
            for (int ch = 0; ch < CH; ch++)
                buffer[i * CH + ch] = 0.0;

            // ------------------------------------------------
            // Calculate Trigger
            // ------------------------------------------------
            for (int j = 0; j < N; j++)
            {
                phase[j] += rate[j] / SAMPLERATE;

                while (phase[j] >= 1.0)
                {
                    modalSynth.updateTriggers(j);
                    phase[j] -= 1.0;
                }
            }

            // ------------------------------------------------
            // Modal Resonators
            // ------------------------------------------------
            modalSynth.process();
            auto& boidOutputs = modalSynth.getOutputs();
            
            // ------------------------------------------------
            // Position Encoding
            // ------------------------------------------------
            std::fill(speakerBuffer.begin(), speakerBuffer.end(), 0.0);
            switch (rendererType)
            {
                case SpatialRendererType::Ambisonics:
                ambi.renderSpeakers(speakerBuffer, boidOutputs, N, spatialParams);
                break;

                case SpatialRendererType::VBAP:
                vbap.renderSpeakers(speakerBuffer, boidOutputs, N, spatialParams);
                break;
            }

            // ------------------------------------------------
            // Output
            // ------------------------------------------------
            outputStage.processSample(&buffer[i * CH], CH, speakerBuffer, drive, master);
        }
    }

    // ========================================================
    // STATE
    // ========================================================
    int Nmax = 0;
    const std::vector<Boid>* boidsPtr = nullptr;
    std::vector<double> phase;

    // ========================================================
    // MODAL SYNTHESIS
    // ========================================================
    ModalSynthesis modalSynth;


    // ========================================================
    // AMBISONIC SPATIALIZATION
    // ========================================================
    AmbisonicRenderer ambi;
    std::vector<SpatialParams> spatialParams;

    // ========================================================
    // VBAP
    // ========================================================
    VBAPRenderer vbap;

    // ========================================================
    // CHOOSE BETWEEN AMBISONICS || VBAP
    // ========================================================
    SpatialRendererType rendererType = SpatialRendererType::VBAP;

    // ========================================================
    // TRIGGER ENGINE
    // ========================================================
    TriggerEngine triggerEngine;

    // ========================================================
    // CONTROL MAPPING
    // ========================================================
    ControlMapping control;

    // ========================================================
    // SPATIAL FEATURES
    // ========================================================
    BoidSpatialSystem spatialSystem;

    // ========================================================
    // AUDIO TEMP BUFFERS
    // ========================================================
    std::vector<double> speakerBuffer;

    // ========================================================
    // OUTPUT
    // ========================================================
    OutputStage outputStage;
};