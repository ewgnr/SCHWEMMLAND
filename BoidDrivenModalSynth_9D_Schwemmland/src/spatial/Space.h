#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>

namespace Space
{
    // -----------------------------------------------------------------
    // Constants (double precision)
    // -----------------------------------------------------------------
    // * Period        → size of toroidal space (wrap domain)
    // * AnglePeriod   → 2π for angular wrapping
    // * WorldScale    → visualization scaling (UI only)
    // * DistanceScale → reference scale for normalization (0..1 mapping)
    // -----------------------------------------------------------------
    inline constexpr double Period = 2.0;
    inline constexpr double AnglePeriod = 2.0 * glm::pi<double>();
    inline constexpr double WorldScale = 50.0;
    inline constexpr double DistanceScale = 0.5;
    inline constexpr double AzimuthScale = 1.0;


    // -----------------------------------------------------------------
    // Toroidal space operations
    // -----------------------------------------------------------------

    /** Wrap a 3D position into the fundamental torus cell [-1, +1). */
    inline glm::dvec3 wrap(const glm::dvec3& v) noexcept
    {
        return v - glm::round(v / Period) * Period;
    }

    /** Compute shortest toroidal difference between two positions. */
    inline glm::dvec3 toroidalDiff(
        const glm::dvec3& a,
        const glm::dvec3& b) noexcept
    {
        glm::dvec3 d = a - b;
        d -= glm::round(d / Period) * Period;
        return d;
    }

    /** Compute toroidal distance (length of shortest wrapped vector). */
    inline double toroidalDistance(
        const glm::dvec3& a,
        const glm::dvec3& b) noexcept
    {
        return glm::length(toroidalDiff(a, b));
    }

    /** Same as toroidalDistance but avoids zero by clamping. */
    inline double toroidalDistanceSafe(
        const glm::dvec3& a,
        const glm::dvec3& b) noexcept
    {
        return std::max(toroidalDistance(a, b), 1e-6);
    }


    // -----------------------------------------------------------------
    // Distance normalization & shaping
    // -----------------------------------------------------------------

    /** Normalize distance between two points to range [0, 1]. */
    inline double normalizedDistance(
        const glm::dvec3& a,
        const glm::dvec3& b,
        double scale) noexcept
    {
        return std::clamp(toroidalDistance(a, b) / scale, 0.0, 1.0);
    }

    /** Normalize distance of a single point from origin. */
    inline double radialNorm(const glm::dvec3& p) noexcept
    {
        return std::clamp(glm::length(p) / DistanceScale, 0.0, 1.0);
    }

    /** Apply exponential-like perceptual gain curve to normalized distance. */
    inline double radialGain(double normDist) noexcept
    {
        return 1.0 - std::exp(-normDist);
    }

    /** Apply exponent shaping to normalized distance (e.g. for pitch mapping). */
    inline double radialCurve(double normDist, double exponent) noexcept
    {
        return std::pow(normDist, std::max(0.2, exponent));
    }

    /** Full perceptual distance pipeline (distance → normalized → shaped). */
    inline double perceptualDistance(
        const glm::dvec3& a,
        const glm::dvec3& b,
        double scale,
        double exponent) noexcept
    {
        double n = normalizedDistance(a, b, scale);
        return radialCurve(n, exponent);
    }


    // -----------------------------------------------------------------
    // Motion / velocity interpretation
    // -----------------------------------------------------------------

    /** Convert velocity vector into perceptual motion energy (0..~1). */
    inline double motionEnergy(const glm::dvec3& vel) noexcept
    {
        double e = glm::length(vel);
        e = std::min(e, 5.0);     // clamp extreme spikes
        return std::tanh(e);      // smooth perceptual response
    }


    // -----------------------------------------------------------------
    // Angular helpers
    // -----------------------------------------------------------------

    /** Compute shortest signed angular difference (wraps around 2π). */
    inline double angularDiff(double from, double to) noexcept
    {
        double d = to - from;
        d -= std::round(d / AnglePeriod) * AnglePeriod;
        return d;
    }

    /** Unwrap angle to produce continuous (non-jumping) angle values. */
    inline double unwrapAngle(double prev, double cur) noexcept
    {
        return prev + angularDiff(prev, cur);
    }

    /** Compute azimuth angle (XZ plane, Y ignored). */
    inline double azimuth(const glm::dvec3& p) noexcept
    {
        return std::atan2(p.x, p.z);
    }

    /** Compute spherical coordinates (azimuth + elevation). */
    inline glm::dvec2 spherical(const glm::dvec3& p) noexcept
    {
        const double r = glm::length(p);
        const double az = std::atan2(p.x, p.z);
        const double el = std::asin(p.y / r);
        return { az, el };
    }


    // -----------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------

    /** Safe vector length (never returns zero). */
    inline double safeLength(const glm::dvec3& v) noexcept
    {
        return std::max(glm::length(v), 1e-6);
    }

    /** Scale position for visualization purposes only. */
    inline glm::dvec3 worldScale(const glm::dvec3& v) noexcept
    {
        return v * WorldScale;
    }

} // namespace Space