#pragma once

#include <vector>
#include <utility>
#include <algorithm>

class LinearEnvelope
{
public:
    explicit LinearEnvelope()
        : currentTime(0.0)
        , startValue(0.0)
        , currentSegment(0)
        , active(false) {
    }

    inline void set(const std::vector<std::pair<double, double>>& pSegments)
    {
        segments = pSegments;
        currentTime = 0.0;
        currentSegment = 0;
        startValue = 0.0;
        active = !segments.empty();
    }

    inline void reset()
    {
        currentTime = 0.0;
        currentSegment = 0;
        startValue = 0.0;
        active = !segments.empty();
    }

    inline double process(double pDt)
    {
        if (!active || segments.empty())
            return 0.0;

        while (pDt > 0.0 && active)
        {
            const auto [segDuration, segTarget] = segments[currentSegment];

            if (segDuration <= 0.0)
            {
                startValue = segTarget;
                currentTime = 0.0;
                ++currentSegment;

                if (currentSegment >= segments.size())
                {
                    active = false;
                    return startValue;
                }

                continue;
            }

            const double remaining = segDuration - currentTime;
            const double step = std::min(pDt, remaining);

            currentTime += step;
            pDt -= step;

            const double t = std::clamp(currentTime / segDuration, 0.0, 1.0);
            const double value = startValue + (segTarget - startValue) * t;

            if (currentTime >= segDuration)
            {
                startValue = segTarget;
                currentTime = 0.0;
                ++currentSegment;

                if (currentSegment >= segments.size())
                {
                    active = false;
                    return startValue;
                }

                if (pDt <= 0.0)
                    return value;

                continue;
            }
            return value;
        }
        return startValue;
    }

    inline bool isActive() const { return active; }

private:
    std::vector<std::pair<double, double>> segments;
    double currentTime = 0.0;
    double startValue = 0.0;
    size_t currentSegment = 0;
    bool active = false;
};
