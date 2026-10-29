#pragma once

#include <atomic>
#include <vector>

//==============================================================
// AUDIO PARAMETERS
//==============================================================

// --- ENVELOPE / ADSR ---
inline std::atomic<double> ENV_ATTACK_MS{ 3.0 };
inline std::atomic<double> ENV_DECAY_MS{ 20.0 };
inline std::atomic<double> ENV_SUSTAIN_LEVEL{ 0.10000000149011612 };
inline std::atomic<double> ENV_SUSTAIN_MS{ 299.3500061035156 };
inline std::atomic<double> ENV_RELEASE_MS{ 135.91000366210938 };

// --- TRIGGER SYSTEM ---
inline std::atomic<double> TRIGGER_MIX{ 0.9999995231628418 };
inline std::atomic<double> TRIGGER_MIN_RATE{ 1.16046142578125 };
inline std::atomic<double> TRIGGER_MAX_RATE{ 7655.65869140625 };
inline std::atomic<double> TRIGGER_CURVE_EXPONENT{ 2.0 };
inline std::atomic<double> TRIGGER_ACTIVITY_GAIN{ 2.2899975776672363 };
inline std::atomic<double> ABSOLUTE_DISTANCE_SCALE{ 773.8124389648438 };

// --- FREQUENCY MAPPING ---
inline std::atomic<double> FREQ_MIN{ 110.0 };
inline std::atomic<double> FREQ_MAX{ 440.0 };
inline std::atomic<double> FREQ_DENSITY_INFLUENCE{ 2.9999947547912598 };
inline std::atomic<double> FREQ_TRANSPOSE{ 0.2900020480155945 };
inline std::atomic<double> FREQ_RADIAL_EXPONENT{ 1.3099216222763062 };
inline std::atomic<double> FREQ_SMOOTHING{ 0.8799999952316284 };

// --- MODAL STRUCTURE ---
inline std::atomic<double> MODE_FREQ_SPREAD{ 0.10000000149011612 };
inline std::atomic<double> MODE_BW_BASE{ 211.4326171875 };
inline std::atomic<double> MODE_AMP_DECAY{ 0.7900000214576721 };
inline std::atomic<double> MODE_BRIGHTNESS{ 2.75 };
inline std::atomic<double> MODE_AMP_BASE{ 0.9999994039535522 };

// --- SPATIAL RENDERING ---
inline std::atomic<double> SPATIAL_GAIN_EXP{ 0.12000727653503418 };
inline std::atomic<double> SPATIAL_DISTANCE_MIN{ 0.010000013746321201 };
inline std::atomic<double> SPATIAL_DISTANCE_MAX{ 4.95999813079834 };
inline std::atomic<double> SPATIAL_WIDTH{ 0.5312082171440125 };
inline std::atomic<double> SPATIAL_DENSITY_EXP_SCALE{ 4.631319046020508 };

// --- OUTPUT STAGE ---
inline std::atomic<double> OUTPUT_TANH_DRIVE{ 4.460000038146973 };
inline std::atomic<double> OUTPUT_MASTER_GAIN{ 0.4300000071525574 };


//==============================================================
// PARAM REGISTRY
//==============================================================

struct ParamEntry
{
    const char* name;
    std::atomic<double>* value;
};


//==============================================================
// GLOBAL PARAMETER LIST
//==============================================================

inline std::vector<ParamEntry> gAudioParams =
{
    // --- ADSR ---
    {"ENV_ATTACK_MS",       &ENV_ATTACK_MS},
    {"ENV_DECAY_MS",        &ENV_DECAY_MS},
    {"ENV_SUSTAIN_LEVEL",   &ENV_SUSTAIN_LEVEL},
    {"ENV_SUSTAIN_MS",      &ENV_SUSTAIN_MS},
    {"ENV_RELEASE_MS",      &ENV_RELEASE_MS},

    // --- TRIGGER ---
    {"TRIGGER_MIX",               &TRIGGER_MIX},
    {"TRIGGER_MIN_RATE",          &TRIGGER_MIN_RATE},
    {"TRIGGER_MAX_RATE",          &TRIGGER_MAX_RATE},
    {"TRIGGER_CURVE_EXPONENT",    &TRIGGER_CURVE_EXPONENT},
    {"TRIGGER_ACTIVITY_GAIN",     &TRIGGER_ACTIVITY_GAIN},
    {"ABSOLUTE_DISTANCE_SCALE",   &ABSOLUTE_DISTANCE_SCALE},

    // --- FREQUENCY ---
    {"FREQ_MIN",                  &FREQ_MIN},
    {"FREQ_MAX",                  &FREQ_MAX},
    {"FREQ_DENSITY_INFLUENCE",    &FREQ_DENSITY_INFLUENCE},
    {"FREQ_TRANSPOSE",            &FREQ_TRANSPOSE},
    {"FREQ_RADIAL_EXPONENT",      &FREQ_RADIAL_EXPONENT},
    {"FREQ_SMOOTHING",            &FREQ_SMOOTHING},

    // --- MODAL ---
    {"MODE_FREQ_SPREAD",          &MODE_FREQ_SPREAD},
    {"MODE_BW_BASE",              &MODE_BW_BASE},
    {"MODE_AMP_DECAY",            &MODE_AMP_DECAY},
    {"MODE_BRIGHTNESS",           &MODE_BRIGHTNESS},
    {"MODE_AMP_BASE",             &MODE_AMP_BASE},

    // --- SPATIAL ---
    {"SPATIAL_GAIN_EXP",          &SPATIAL_GAIN_EXP},
    {"SPATIAL_DISTANCE_MIN",      &SPATIAL_DISTANCE_MIN},
    {"SPATIAL_DISTANCE_MAX",      &SPATIAL_DISTANCE_MAX},
    {"SPATIAL_WIDTH",             &SPATIAL_WIDTH},
    {"SPATIAL_DENSITY_EXP_SCALE", &SPATIAL_DENSITY_EXP_SCALE},

    // --- OUTPUT ---
    {"OUTPUT_TANH_DRIVE",         &OUTPUT_TANH_DRIVE},
    {"OUTPUT_MASTER_GAIN",        &OUTPUT_MASTER_GAIN},
};