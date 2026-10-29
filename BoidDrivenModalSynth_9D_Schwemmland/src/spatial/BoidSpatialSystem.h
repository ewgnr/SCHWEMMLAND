#pragma once

#include "ParamRegistry.h"
#include "Boid.h"
#include "Space.h"

#include <vector>
#include <array>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <glm/glm.hpp>

// ============================================================
// SUBSPACE
// ============================================================
struct Subspace
{
    std::array<int, 3> dims;
    double weight = 1.f;
};

// ============================================================
// SPATIAL FRAME (single view)
// ============================================================
struct SpatialFrame
{
    int N = 0;

    std::vector<glm::vec3> wrappedPos;
    std::vector<glm::vec3> normalizedPos;
    std::vector<glm::vec3> continuousPos;

    std::vector<glm::vec3> velocity;
    std::vector<glm::vec3> continuousVelocity;

    std::vector<float> density;
    std::vector<float> meanDistance;

    double globalMeanDistance = 1.0;

    void allocate(int N)
    {
        wrappedPos.resize(N);
        normalizedPos.resize(N);
        continuousPos.resize(N);
        velocity.resize(N);
        continuousVelocity.resize(N);
        density.resize(N);
        meanDistance.resize(N);
    }
};

// ============================================================
// MULTI FRAME (3 parallel projections)
// ============================================================
struct MultiSpatialFrame
{
    SpatialFrame sub[3];

    void allocate(int maxN)
    {
        for (auto& s : sub)
            s.allocate(maxN);
    }
};

// ============================================================
// BOID AGGREGATOR (OPTIMIZED SINGLE SOURCE OF TRUTH)
// ============================================================
class BoidAggregator
{
public:

    void setup(int maxBoids)
    {
        spatialPos.resize(maxBoids);
        prevSpatialPos.resize(maxBoids);
        densities.resize(maxBoids, 0.f);
        meanDistances.resize(maxBoids, 1.f);
    }

    void setSubspaces(const std::vector<Subspace>& s)
    {
        subspaces = s;
        normalizeWeights();
    }

    void update(const std::vector<Boid>& boids)
    {
        const int N = (int)boids.size();
        if (N == 0) return;

        computeSpatialPositions(boids);
        computeMetrics(N);
    }

    inline glm::vec3 project(const glm::vec3& p, const Subspace&) const
    {
        return p;
    }

    void writeFrame(SpatialFrame& frame, int N, const Subspace& s) const
    {
        frame.N = N;

        for (int i = 0; i < N; i++)
        {
            const glm::dvec3 p = project(spatialPos[i], s);

            frame.wrappedPos[i] = p;
            frame.continuousPos[i] = spatialPos[i];
            frame.normalizedPos[i] = (p - normCenter) * normScale;

            frame.velocity[i] = getVelocity(i);
            frame.continuousVelocity[i] = frame.velocity[i];

            frame.density[i] = densities[i];
            frame.meanDistance[i] = meanDistances[i];
        }

        frame.globalMeanDistance = globalMeanDistance;
    }

private:

    // -------- data --------
    std::vector<Subspace> subspaces;

    std::vector<glm::vec3> spatialPos;
    std::vector<glm::vec3> prevSpatialPos;

    std::vector<double> densities;
    std::vector<double> meanDistances;

    glm::dvec3 normCenter = glm::vec3(0.0f);
    double normScale = 1.0;

    double globalMeanDistance = 1.0;

    // -------- weights --------
    void normalizeWeights()
    {
        float sum = 0.f;
        for (auto& s : subspaces) sum += s.weight;
        if (sum > 0.f)
            for (auto& s : subspaces) s.weight /= sum;
    }

    glm::vec3 extract(const Boid& b) const
    {
        glm::vec3 agg(0.f);
        for (const auto& s : subspaces)
            agg += s.weight * glm::vec3(
                b.position[s.dims[0]],
                b.position[s.dims[1]],
                b.position[s.dims[2]]
            );
        return agg;
    }

    void computeSpatialPositions(const std::vector<Boid>& boids)
    {
        std::swap(prevSpatialPos, spatialPos);

        for (size_t i = 0; i < boids.size(); i++)
        {
            glm::vec3 curr = Space::wrap(extract(boids[i]));
            glm::vec3 prev = Space::wrap(prevSpatialPos[i]);

            glm::vec3 delta = Space::toroidalDiff(curr, prev);
            spatialPos[i] = prev + delta;
        }
    }

    void computeMetrics(int N)
    {
        std::fill(meanDistances.begin(), meanDistances.begin() + N, 0.0);

        double sum = 0.0;
        int count = 0;

        for (int i = 0; i < N; i++)
            for (int j = i + 1; j < N; j++)
            {
                glm::vec3 d = Space::toroidalDiff(spatialPos[i], spatialPos[j]);
                float dist = glm::length(d);

                meanDistances[i] += dist;
                meanDistances[j] += dist;

                sum += dist;
                count++;
            }

        const double inv = 1.0 / std::max(1, N - 1);

        for (int i = 0; i < N; i++)
        {
            double md = meanDistances[i] * inv;
            meanDistances[i] = md;
            densities[i] = std::exp(-md * SPATIAL_DENSITY_EXP_SCALE.load(std::memory_order_relaxed));
        }

        globalMeanDistance = (count > 0) ? sum / count : 1.0;
    }

    glm::vec3 getVelocity(int i) const
    {
        return Space::toroidalDiff(Space::wrap(spatialPos[i]), Space::wrap(prevSpatialPos[i]));
    }
};

// ============================================================
// SYSTEM WRAPPER (LOSSLESS + FAST)
// ============================================================
class BoidSpatialSystem
{
public:

    void setup(int maxBoids)
    {
        Nmax = maxBoids;

        aggregator.setup(maxBoids);

        subspaces[0] = { {0,1,2}, 1.0 };
        subspaces[1] = { {3,4,5}, 1.0 };
        subspaces[2] = { {6,7,8}, 1.0 };

        aggregator.setSubspaces({ subspaces, subspaces + 3 });

        a.allocate(Nmax);
        b.allocate(Nmax);

        active.store(&a, std::memory_order_release);
    }

    void update(const std::vector<Boid>& boids)
    {
        const int N = (int)boids.size();

        MultiSpatialFrame* out = (active.load(std::memory_order_relaxed) == &a) ? &b : &a;

        if (N == 0)
        {
            for (int i = 0; i < 3; i++)
                out->sub[i].N = 0;

            out->sub[0].globalMeanDistance = 1.0;
            active.store(out, std::memory_order_release);
            return;
        }

        aggregator.update(boids);

        for (int i = 0; i < 3; i++)
            aggregator.writeFrame(out->sub[i], N, subspaces[i]);

        active.store(out, std::memory_order_release);
    }

    const MultiSpatialFrame* getFrame() const
    {
        return active.load(std::memory_order_acquire);
    }

    const SpatialFrame* getGeometryFrame() const
    {
        return &getFrame()->sub[0];
    }

    const SpatialFrame* getMotionFrame() const
    {
        return &getFrame()->sub[1];
    }

    const SpatialFrame* getDensityFrame() const
    {
        return &getFrame()->sub[2];
    }

private:

    int Nmax = 0;
    BoidAggregator aggregator;
    Subspace subspaces[3];
    MultiSpatialFrame a, b;
    std::atomic<MultiSpatialFrame*> active;
};