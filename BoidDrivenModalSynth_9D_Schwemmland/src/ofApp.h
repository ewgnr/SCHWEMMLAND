#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"

#include "SwarmOSCReceiver.h"
#include "BoidSoundEngine.h"

#include "ParamRegistry.h"
#include "TimelineEngine.h"

class ofApp : public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void draw() override;

    void onSliderEvent(ofxDatGuiSliderEvent e);
    void syncGuiToParams();

    void saveAudioPreset(const std::string& presetName);
    void loadAudioPreset(const std::string& presetName);
    void loadAllAudioPresets();
    void keyPressed(int key);

    ofSoundStream soundStream;
    SwarmOSCReceiver swarm;
    AdaptiveBoidSoundEngine engine;
    ofEasyCam cam;

    ofxDatGui* gui = nullptr;

    ofxDatGuiFolder* fTrigger = nullptr;
    ofxDatGuiFolder* fFrequency = nullptr;
    ofxDatGuiFolder* fModal = nullptr;
    ofxDatGuiFolder* fSpatial = nullptr;
    ofxDatGuiFolder* fOutput = nullptr;

    struct SliderBinding
    {
        std::string name;
        ofxDatGuiSlider* slider = nullptr;
        std::atomic<double>* parameter = nullptr;
    };

    std::vector<SliderBinding> sliderBindings;

    ofxDatGuiSlider* addBoundSlider(ofxDatGuiFolder* folder,
        const std::string& label,
        float min,
        float max,
        float value,
        std::atomic<double>& param);

    enum PresetMode
    {
        NONE,
        LOAD_SELECT_MODE,
        SAVE_MODE,
        LOAD_MODE
    };

    PresetMode presetMode = NONE;

    int selectedPreset = 1;
    const int maxPresets = 20;

    enum ParamMode
    {
        EDIT_MODE,
        TIMELINE_MODE
    };

    ParamMode paramMode = TIMELINE_MODE;

    TimelineEngine timeline;
    std::vector<TimelineEngine::AudioParam> timelineParams;

    /*                              001   002   003   004   005   006   007   008   009 */
    std::vector<int> presetYears = { 1750, 1810, 1850, 1880, 1914, 1950, 1980, 1990, 2020 };

    float yearToPresetPosition(int year);
};