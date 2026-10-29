#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <atomic>

#include <glm/glm.hpp>

#include "BoidSpatialSystem.h"
#include "ParamRegistry.h"
#include "ModalParams.h"

struct ControlFrame
{
    ModalParamsBuffer buffer;
    std::vector<double> driveField;
    int N = 0;
};

class ControlMapping
{
public:

    void setup(int maxBoids)
    {
        Nmax = maxBoids;

        smoothedFreq.resize(Nmax, 0.0);
        smoothedEnergy.resize(Nmax, 0.0);

        for (int i = 0; i < 2; i++)
        {
            frames[i].buffer.perBoid.resize(Nmax);
            frames[i].driveField.resize(Nmax, 0.0);

            for (auto& b : frames[i].buffer.perBoid)
            {
                b.freq.resize(maxBoids);
                b.bw.resize(maxBoids);
                b.amp.resize(maxBoids);
            }
        }
    }

    void update(const SpatialFrame& geometry,
        const SpatialFrame& motion,
        const SpatialFrame& density,
        int N)
    {
        if (N == 0) return;

        computeCenter(geometry, N);
        computeEnergy(motion, N);
        int w = writeIndex.load(std::memory_order_relaxed);
        auto& frame = frames[w];

        frame.N = N;

        computeModes(frame, geometry, motion, density, N);

        int newRead = w;
        int newWrite = 1 - w;

        readIndex.store(newRead, std::memory_order_release);
        writeIndex.store(newWrite, std::memory_order_relaxed);
    }

    const ControlFrame* getFrame() const
    {
        int r = readIndex.load(std::memory_order_acquire);
        return &frames[r];
    }

private:

    int Nmax = 0;

    std::vector<double> smoothedFreq;
    std::vector<double> smoothedEnergy;

    glm::dvec3 smoothedCenter = glm::dvec3(0.0);

    ControlFrame frames[2];

    std::atomic<int> writeIndex{ 0 };
    std::atomic<int> readIndex{ 1 };

    // ========================================================
    // A) CENTER (shared reference frame)
    // ========================================================
    void computeCenter(const SpatialFrame& geometry, int N)
    {
        glm::dvec3 ref = geometry.wrappedPos[0];
        glm::dvec3 center(0.0);

        for (int i = 0; i < N; i++)
        {
            glm::dvec3 p = geometry.wrappedPos[i];
            glm::dvec3 d = p - ref;
            d -= glm::round(d * 0.5) * 2.0;
            center += ref + d;
        }

        center /= double(N);

        const double alpha = 0.02;
        smoothedCenter = (1.0 - alpha) * smoothedCenter + alpha * center;
    }

    // ========================================================
    // B) MOTION → ENERGY FIELD
    // ========================================================
    void computeEnergy(const SpatialFrame& motion, int N)
    {
        const double energySmooth = 0.05;

        for (int i = 0; i < N; i++)
        {
            glm::dvec3 vel = motion.continuousVelocity[i];

            double e = glm::length(vel);
            e = std::min(e, 5.0);
            e = std::tanh(e);

            if (smoothedEnergy[i] <= 0.0)
                smoothedEnergy[i] = e;
            else
                smoothedEnergy[i] = (1.0 - energySmooth) * smoothedEnergy[i] + energySmooth * e;
        }
    }

    // ========================================================
    // C) FMAP + SYNTHESIS
    // ========================================================
    void computeModes(ControlFrame& frame, const SpatialFrame& geometry, const SpatialFrame& motion, const SpatialFrame& density, int N)
    {
        const double densSmooth = 0.12;

        const double fmin = FREQ_MIN.load(std::memory_order_relaxed);
        const double fmax = FREQ_MAX.load(std::memory_order_relaxed);
        const double transpose = FREQ_TRANSPOSE.load(std::memory_order_relaxed);
        const double freqDensityInfluence = FREQ_DENSITY_INFLUENCE.load(std::memory_order_relaxed);
        const double freqRadialExponent = FREQ_RADIAL_EXPONENT.load(std::memory_order_relaxed);

        const double bwMod = MODE_BW_BASE.load(std::memory_order_relaxed);
        const double spreadMod = MODE_FREQ_SPREAD.load(std::memory_order_relaxed);
        const double ampBase = MODE_AMP_BASE.load(std::memory_order_relaxed);
        const double decayMod = MODE_AMP_DECAY.load(std::memory_order_relaxed);
        const double brightness = MODE_BRIGHTNESS.load(std::memory_order_relaxed);

        const double spatialScale = 0.5;

        const double logMin = std::log(fmin);
        const double logMax = std::log(fmax);
        const double logRange = logMax - logMin;

        const double brightnessClamped = std::clamp(brightness, 0.1, 3.0);

        for (int i = 0; i < N; i++)
        {
            // -----------------------------
            // density-modulated excitation field (continuous activity proxy)
            // -----------------------------
            double dens = density.density[i];
            double e = smoothedEnergy[i] * std::exp(-dens);
            e = std::tanh(e * 1.5);

            frame.driveField[i] = e;

            auto& out = frame.buffer.perBoid[i];

            // -----------------------------
            // FMAP (position → frequency)
            // -----------------------------
            glm::dvec3 d = Space::toroidalDiff(geometry.wrappedPos[i], smoothedCenter);
            double dist = std::max(glm::length(d), 1e-6);

            double normDist = std::clamp(dist / spatialScale, 0.0, 1.0);
            double t = std::pow(normDist, std::max(0.2, freqRadialExponent));

            double base = fmin * std::pow(fmax / fmin, t);
            base *= (1.0 + freqDensityInfluence * density.density[i]);
            base *= transpose;

            double norm = (std::log(base) - logMin) / logRange;
            base = std::exp(logMin + std::clamp(norm, 0.0, 1.0) * logRange);

            // smoothing
            double alpha = (smoothedFreq[i] <= 0.0) ? 1.0 : (1.0 - densSmooth);

            smoothedFreq[i] = (smoothedFreq[i] <= 0.0)
                ? base
                : (1.0 - alpha) * smoothedFreq[i] + alpha * base;

            double freq = smoothedFreq[i];

            // -----------------------------
            // MOTION influence
            // -----------------------------
            double energy = smoothedEnergy[i];
            double chaos = 1.0 + energy;

            // -----------------------------
            // MODE SYNTHESIS
            // -----------------------------
            for (size_t m = 0; m < out.freq.size(); m++)
            {
                double M = out.freq.size();
                double tM = (M > 1) ? double(m) / (M - 1) : 0.0;

                double freqFactor = std::pow(1.0 + spreadMod, tM);
                double bwFactor = std::pow(1.3, tM);
                double ampFactor = std::pow(0.7, tM);

                out.freq[m] = freq * freqFactor * (1.0 + 0.02 * chaos * std::sin((double)m));

                out.bw[m] = bwMod * bwFactor;

                out.amp[m] = (ampBase / (1.0 + decayMod * m)) * std::pow(1.0 + energy, -m * 0.5) * std::pow(brightnessClamped, m * 0.5) * ampFactor;
            }
        }
    }
};