#pragma once

#include "ofMain.h"
#include <unordered_map>
#include <vector>
#include <string>
#include <atomic>
#include <glm/glm.hpp>
#include <algorithm>

class TimelineEngine
{
public:

    // ------------------------------------------------------------
    // external parameter descriptor
    struct AudioParam
    {
        std::string name;
        std::atomic<double>* value;
    };

    // ------------------------------------------------------------
    // preset structure
    struct Preset
    {
        std::unordered_map<std::string, double> values;
    };

    // ------------------------------------------------------------
    // state
    std::vector<Preset> presets;

    float position = 0.0f;   // continuous index in preset space
    bool snapEnabled = false;
    int  snapIndex = -1;

    // ------------------------------------------------------------
    // bound instrument parameters
    inline void bind(std::vector<AudioParam>& p)
    {
        params = &p;
    }

    inline bool isReady() const
    {
        return params != nullptr && !params->empty();
    }

    // ------------------------------------------------------------
    // setup
    inline void setup(int reserveSize = 0)
    {
        presets.reserve(reserveSize);
        position = 0.0f;
        snapEnabled = false;
        snapIndex = -1;
    }

    inline void clear()
    {
        presets.clear();
        position = 0.0f;
        targetPosition = 0.0f;
        snapEnabled = false;
        snapIndex = -1;
    }

    // ------------------------------------------------------------
    // add preset from JSON
    inline void addPreset(const ofJson& json)
    {
        if (!isReady())
            return;

        if (!json.contains("audio"))
            return;

        Preset p;

        for (auto& param : *params)
        {
            if (json["audio"].contains(param.name))
            {
                p.values[param.name] =
                    json["audio"][param.name].get<double>();
            }
        }

        presets.push_back(std::move(p));
    }

    // ------------------------------------------------------------
    // timeline control
    inline void setPosition(float p)
    {
        if (presets.empty()) return;
        position = std::clamp(p, 0.0f, (float)presets.size() - 1.001f);
    }

    inline void setNormalized(float t)
    {
        if (presets.empty()) return;
        setPosition(t * (presets.size() - 1));
    }

    inline void setSnap(int idx)
    {
        if (presets.empty()) return;

        snapIndex = ofClamp(idx, 0, (int)presets.size() - 1);
        snapEnabled = true;
    }

    inline void clearSnap()
    {
        snapEnabled = false;
        snapIndex = -1;
    }

    // ------------------------------------------------------------
    // update timeline
    inline void update(float dt = 1.0f)
    {
        if (presets.empty())
            return;

        if (snapEnabled)
        {
            position = snapIndex;
            return;
        }

        // smooth movement toward target
        position = ofLerp(position, targetPosition, moveSpeed * dt);
        position = clampPos(position);
    }

    // ------------------------------------------------------------
    // interpolation core
    inline void applyInterpolated()
    {
        if (!isReady() || presets.empty())
            return;

        if (presets.size() == 1)
        {
            applyPreset(presets[0]);
            return;
        }

        int i0 = ofClamp((int)floor(position), 0, (int)presets.size() - 1);
        int i1 = std::min(i0 + 1, (int)presets.size() - 1);

        float t = position - i0;
        t = glm::smoothstep(0.0f, 1.0f, t);

        const Preset& A = presets[i0];
        const Preset& B = presets[i1];

        for (auto& param : *params)
        {
            auto a = A.values.find(param.name);
            auto b = B.values.find(param.name);

            if (a == A.values.end() || b == B.values.end())
                continue;

            double v = ofLerp(a->second, b->second, t);
            param.value->store(v, std::memory_order_relaxed);
        }
    }

    inline void saveCurrentToFile(const std::string& baseName,
        const std::string& presetName,
        const std::vector<AudioParam>& params)
    {
        Preset p;

        for (const auto& param : params)
        {
            if (param.value)
            {
                p.values[param.name] = param.value->load(std::memory_order_relaxed);
            }
        }

        presets.push_back(p);

        ofJson json;

        for (const auto& [name, value] : p.values)
        {
            json["audio"][name] = value;
        }

        std::string fileName = baseName + presetName + ".json";

        ofSavePrettyJson(fileName, json);

        ofLogNotice() << "Saved preset: " << fileName;
    }

    inline float getTime() const { return position; }
    inline void setTime(float t) { setPosition(t); }

    float targetPosition = 0.0f;
    float moveSpeed = 1.0f;

private:

    std::vector<AudioParam>* params = nullptr;

    inline float clampPos(float p) const
    {
        if (presets.empty()) return 0.0f;
        return ofClamp(p, 0.0f, (float)presets.size() - 1.001f);
    }

    inline void applyPreset(const Preset& p)
    {
        if (!isReady()) return;

        for (auto& param : *params)
        {
            auto it = p.values.find(param.name);
            if (it != p.values.end())
            {
                param.value->store(it->second, std::memory_order_relaxed);
            }
        }
    }
};