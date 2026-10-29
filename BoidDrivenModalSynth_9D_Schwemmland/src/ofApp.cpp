#include "ofApp.h"

//--------------------------------------------------------------
ofxDatGuiSlider * ofApp::addBoundSlider(ofxDatGuiFolder * folder,
    const std::string & label,
    float min,
    float max,
    float value,
    std::atomic<double>&param)
{
    auto* s = folder->addSlider(label, min, max, value);
    sliderBindings.push_back({ label, s, &param });
    return s;
}

//--------------------------------------------------------------
void ofApp::setup()
{
    ofSetFrameRate(60);

    constexpr int numberOfBoids = 8;
    swarm.setup(9006, numberOfBoids);
    engine.setup(numberOfBoids);

    ofSoundStreamSettings settings;
    settings.setOutListener(&engine);
    settings.numOutputChannels = NUM_SPEAKERS;
    settings.numInputChannels = 0;
    settings.sampleRate = SAMPLERATE;
    settings.bufferSize = BUFFERSIZE;
    settings.numBuffers = 4;
    auto devices = soundStream.getDeviceList(ofSoundDevice::Api::MS_ASIO);

    for (size_t i = 0; i < devices.size(); ++i)
    {
        const auto& d = devices[i];

        std::cout << "Device " << i << ": " << d.name << std::endl;
        std::cout << "  ID: " << d.deviceID << std::endl;
        std::cout << "  Inputs: " << d.inputChannels << std::endl;
        std::cout << "  Outputs: " << d.outputChannels << std::endl;
        std::cout << "  Default Input: " << d.isDefaultInput << std::endl;
        std::cout << "  Default Output: " << d.isDefaultOutput << std::endl;
        std::cout << std::endl;
    }

    settings.setApi(ofSoundDevice::Api::MS_ASIO);
    soundStream.setup(settings);

    gui = new ofxDatGui(ofxDatGuiAnchor::TOP_LEFT);
    gui->setWidth(1024);
    gui->addHeader("BOID SOUND ENGINE");
    gui->addBreak();

    int totalWidth = 824;
    float labelWidth = 200;

    // --- ENVELOPE ---
    auto* fEnvelope = gui->addFolder("ENVELOPE / ADSR");
    addBoundSlider(fEnvelope, "ATTACK (ms)", 0.020833, 500.0, ENV_ATTACK_MS.load(), ENV_ATTACK_MS);
    addBoundSlider(fEnvelope, "DECAY (ms)", 0.020833, 1000.0, ENV_DECAY_MS.load(), ENV_DECAY_MS);
    addBoundSlider(fEnvelope, "SUSTAIN LEVEL", 0.0, 1.0, ENV_SUSTAIN_LEVEL.load(), ENV_SUSTAIN_LEVEL);
    addBoundSlider(fEnvelope, "SUSTAIN TIME (ms)", 0.0, 2000.0, ENV_SUSTAIN_MS.load(), ENV_SUSTAIN_MS);
    addBoundSlider(fEnvelope, "RELEASE (ms)", 0.0, 2000.0, ENV_RELEASE_MS.load(), ENV_RELEASE_MS);
    fEnvelope->expand();

    // --- TRIGGER ---
    fTrigger = gui->addFolder("TRIGGER / EXCITATION");
    addBoundSlider(fTrigger, "MIX", 0.0, 1.0, TRIGGER_MIX.load(), TRIGGER_MIX);
    addBoundSlider(fTrigger, "MIN RATE", 1.0, 11999.0, TRIGGER_MIN_RATE.load(), TRIGGER_MIN_RATE);
    addBoundSlider(fTrigger, "MAX RATE", 1.0, 12000.0, TRIGGER_MAX_RATE.load(), TRIGGER_MAX_RATE);
    addBoundSlider(fTrigger, "CURVE", 0.1, 2.0, TRIGGER_CURVE_EXPONENT.load(), TRIGGER_CURVE_EXPONENT);
    addBoundSlider(fTrigger, "ACTIVITY GAIN", 0.0, 3.0, TRIGGER_ACTIVITY_GAIN.load(), TRIGGER_ACTIVITY_GAIN);
    addBoundSlider(fTrigger, "ABS DIST SCALE", 50.0, 2000.0, ABSOLUTE_DISTANCE_SCALE.load(), ABSOLUTE_DISTANCE_SCALE);
    fTrigger->expand();

    // --- SPATIAL FIELD ---
    fSpatial = gui->addFolder("SPATIAL FIELD");
    addBoundSlider(fSpatial, "GAIN EXP", 0.1, 6.0, SPATIAL_GAIN_EXP.load(), SPATIAL_GAIN_EXP);
    addBoundSlider(fSpatial, "DIST MIN", 0.01, 1.0, SPATIAL_DISTANCE_MIN.load(), SPATIAL_DISTANCE_MIN);
    addBoundSlider(fSpatial, "DIST MAX", 0.1, 5.0, SPATIAL_DISTANCE_MAX.load(), SPATIAL_DISTANCE_MAX);
    addBoundSlider(fSpatial, "WIDTH", 0.2, 3.0, SPATIAL_WIDTH.load(), SPATIAL_WIDTH);
    addBoundSlider(fSpatial, "DENSITY SCALE", 0.1, 5.0, SPATIAL_DENSITY_EXP_SCALE.load(), SPATIAL_DENSITY_EXP_SCALE);
    fSpatial->expand();

    // --- FREQUENCY FIELD ---
    fFrequency = gui->addFolder("FREQUENCY FIELD");
    addBoundSlider(fFrequency, "FREQ MIN", 20.0, 500.0, FREQ_MIN.load(), FREQ_MIN);
    addBoundSlider(fFrequency, "FREQ MAX", 200.0, 8000.0, FREQ_MAX.load(), FREQ_MAX);
    addBoundSlider(fFrequency, "DENSITY INFL", 0.0, 3.0, FREQ_DENSITY_INFLUENCE.load(), FREQ_DENSITY_INFLUENCE);
    addBoundSlider(fFrequency, "TRANSPOSE", 0.25, 4.0, FREQ_TRANSPOSE.load(), FREQ_TRANSPOSE);
    addBoundSlider(fFrequency, "RADIAL EXP", 0.1, 2.0, FREQ_RADIAL_EXPONENT.load(), FREQ_RADIAL_EXPONENT);
    addBoundSlider(fFrequency, "SMOOTHING", 0.8, 0.999, FREQ_SMOOTHING.load(), FREQ_SMOOTHING);
    fFrequency->expand();

    // --- MODAL ENGINE ---
    fModal = gui->addFolder("MODAL ENGINE");
    addBoundSlider(fModal, "FREQ SPREAD", 0.0, 0.1, MODE_FREQ_SPREAD.load(), MODE_FREQ_SPREAD);
    addBoundSlider(fModal, "BW BASE", 1.0, 3000.0, MODE_BW_BASE.load(), MODE_BW_BASE);
    addBoundSlider(fModal, "AMP DECAY", 0.0, 1.0, MODE_AMP_DECAY.load(), MODE_AMP_DECAY);
    addBoundSlider(fModal, "BRIGHTNESS", 0.1, 3.0, MODE_BRIGHTNESS.load(), MODE_BRIGHTNESS);
    addBoundSlider(fModal, "AMP BASE", 0.0, 1.0, MODE_AMP_BASE.load(), MODE_AMP_BASE);
    fModal->expand();

    // --- OUTPUT STAGE ---
    fOutput = gui->addFolder("OUTPUT STAGE");
    addBoundSlider(fOutput, "TANH DRIVE", 0.5, 10.0, OUTPUT_TANH_DRIVE.load(), OUTPUT_TANH_DRIVE);
    addBoundSlider(fOutput, "MASTER GAIN", 0.0, 2.0, OUTPUT_MASTER_GAIN.load(), OUTPUT_MASTER_GAIN);
    fOutput->expand();

    gui->onSliderEvent(this, &ofApp::onSliderEvent);

    for (auto& p : gAudioParams)
    {
        timelineParams.push_back({ p.name, p.value });
    }

    timeline.bind(timelineParams);
}

//--------------------------------------------------------------
void ofApp::update()
{
    swarm.update();
    engine.updateBoids(swarm.getBoids());

    if (paramMode == TIMELINE_MODE)
    {
        int year = swarm.getTimelineYear();
        // timeline.targetPosition = yearToPresetPosition(year);

        timeline.update(1.0f / 60.0f);
        timeline.applyInterpolated();
        syncGuiToParams();
    }

    if (gui) gui->update();
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackground(0);

    int guiWidth = 1024;
    int rightWidth = ofGetWidth() - guiWidth;
    int height = ofGetHeight();

    ofPushStyle();
    ofSetColor(30);
    ofDrawRectangle(0, 0, guiWidth, height);
    ofPopStyle();

    ofViewport(guiWidth, 0, rightWidth, height);

    cam.begin();

    ofSetColor(80, 200, 255);
    
    for (const auto& b : engine.getBoids())
    {
        glm::dvec3 dPos = Space::worldScale(glm::dvec3(b.position[0], b.position[1], b.position[2]));

        ofDrawSphere(dPos, 2.0);
    }

    cam.end();

    ofViewport(0, 0, ofGetWidth(), ofGetHeight());

    gui->draw();
}

//--------------------------------------------------------------
void ofApp::onSliderEvent(ofxDatGuiSliderEvent e)
{
    for (auto& binding : sliderBindings)
    {
        if (e.target == binding.slider)
        {
            binding.parameter->store(e.value);
            break;
        }
    }
}

//--------------------------------------------------------------
void ofApp::syncGuiToParams()
{
    for (auto& binding : sliderBindings)
    {
        if (binding.slider && binding.parameter)
        {
            binding.slider->setValue(binding.parameter->load(std::memory_order_relaxed));
        }
    }
}

//--------------------------------------------------------------
void ofApp::saveAudioPreset(const std::string& presetName)
{
    timeline.saveCurrentToFile("preset_", presetName, timelineParams);
}

//--------------------------------------------------------------
void ofApp::loadAudioPreset(const std::string& presetName)
{
    std::string fileName = "preset_" + presetName + ".json";

    if (!ofFile::doesFileExist(fileName))
        return;

    ofJson json = ofLoadJson(fileName);

    if (!json.contains("audio"))
        return;

    TimelineEngine::Preset p;

    for (auto& param : gAudioParams)
    {
        if (json["audio"].contains(param.name))
        {
            double value = json["audio"][param.name].get<double>();

            param.value->store(value, std::memory_order_relaxed);
            p.values[param.name] = value;
        }
    }

    timeline.presets.push_back(std::move(p));
}

//--------------------------------------------------------------
void ofApp::loadAllAudioPresets()
{
    timeline.presets.clear();

    ofDirectory dir;
    dir.listDir(".");
    dir.sort();

    for (auto& file : dir.getFiles())
    {
        if (file.getExtension() != "json")
            continue;

        if (!ofIsStringInString(file.getBaseName(), "preset_"))
            continue;

        ofJson json = ofLoadJson(file.getAbsolutePath());

        if (!json.contains("audio"))
            continue;

        TimelineEngine::Preset p;

        for (auto& param : gAudioParams)
        {
            if (json["audio"].contains(param.name))
            {
                p.values[param.name] = json["audio"][param.name].get<double>();
            }
        }

        timeline.presets.push_back(std::move(p));
    }

    ofLogNotice() << "Loaded presets: " << timeline.presets.size();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    // Enter save mode
    if (key == 's')
    {
        presetMode = SAVE_MODE;
        selectedPreset = 1;

        std::cout << "\n=== SAVE PRESET ===\n";
        std::cout << "Existing presets:\n";

        ofDirectory dir;
        dir.listDir(".");
        dir.sort();

        for (auto& file : dir.getFiles())
        {
            std::string name = file.getFileName();

            if (ofIsStringInString(name, "preset_") &&
                file.getExtension() == "json")
            {
                std::cout << "  " << name << std::endl;
            }
        }

        std::cout << "\nTimeline presets (current session):\n";

        if (timeline.presets.empty())
        {
            std::cout << "  (none)\n";
        }
        else
        {
            for (size_t i = 0; i < timeline.presets.size(); i++)
            {
                std::stringstream ss;
                ss << std::setw(3) << std::setfill('0') << (i + 1);

                std::cout << "  [" << ss.str() << "] preset\n";
            }
        }

        std::cout << "Use UP/DOWN arrows, ENTER to save.\n";
        std::cout << "Selected: " << std::setw(3) << std::setfill('0') << selectedPreset << std::endl;
        return;
    }

    // Enter load mode
    if (key == 'l')
    {
        presetMode = LOAD_SELECT_MODE;

        std::cout << "\n=== LOAD ===\n";
        std::cout << "[o] Single preset\n";
        std::cout << "[a] All presets\n";

        return;
    }

    if (presetMode == LOAD_SELECT_MODE)
    {
        if (key == 'a')
        {
            loadAllAudioPresets();
            presetMode = NONE;

            std::cout << "Loaded all presets\n";
        }

        if (key == 'o')
        {
            presetMode = LOAD_MODE;
            selectedPreset = 1;

            std::cout << "\n=== LOAD PRESET ===\n";
            std::cout << "Use UP/DOWN arrows, ENTER to load.\n";
            std::cout << "Selected: " << std::setw(3) << std::setfill('0') << selectedPreset << std::endl;
        }
    }

    if (presetMode == SAVE_MODE || presetMode == LOAD_MODE)
    {
        if (key == OF_KEY_UP)
        {
            selectedPreset++;

            if (selectedPreset > maxPresets)
                selectedPreset = 1;

            std::cout << "Selected: " << std::setw(3) << std::setfill('0') << selectedPreset << std::endl;
        }

        if (key == OF_KEY_DOWN)
        {
            selectedPreset--;

            if (selectedPreset < 1)
                selectedPreset = maxPresets;

            std::cout << "Selected: " << std::setw(3) << std::setfill('0') << selectedPreset << std::endl;
        }

        if (key == OF_KEY_RETURN)
        {
            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << selectedPreset;

            std::string presetName = ss.str();

            if (presetMode == SAVE_MODE)
            {
                saveAudioPreset(presetName);
                std::cout << "Saved preset " << presetName << std::endl;
            }
            else if (presetMode == LOAD_MODE)
            {
                loadAudioPreset(presetName);
                syncGuiToParams();

                std::cout << "Loaded preset " << presetName << std::endl;
            }

            presetMode = NONE;
        }

        if (key == 'q')
        {
            presetMode = NONE;
            std::cout << "Cancelled\n";
        }
    }

    if (key == 't')
    {
        paramMode = TIMELINE_MODE;
        std::cout << "Timeline mode ON\n";
    }

    if (key == 'e')
    {
        paramMode = EDIT_MODE;
        timeline.clearSnap();
        std::cout << "Edit mode ON\n";
    }

    if (paramMode == TIMELINE_MODE)
    {
        if (key == OF_KEY_RIGHT)
        {
            timeline.clearSnap();
            timeline.targetPosition = timeline.getTime() + 1.0f;
        }

        if (key == OF_KEY_LEFT)
        {
            timeline.clearSnap();
            timeline.targetPosition = timeline.getTime() - 1.0f;
        }
    }

    if (key == 'r')
    {
        timeline.clear();
        std::cout << "[TIMELINE] Reset complete\n";
    }
}

float ofApp::yearToPresetPosition(int year)
{
    if (presetYears.empty())
        return 0.0f;

    if (year <= presetYears.front())
        return 0.0f;

    if (year >= presetYears.back())
        return presetYears.size() - 1;

    for (size_t i = 0; i < presetYears.size() - 1; i++)
    {
        int y0 = presetYears[i];
        int y1 = presetYears[i + 1];

        if (year >= y0 && year <= y1)
        {
            float t = float(year - y0) / float(y1 - y0);

            return i + t;
        }
    }

    return 0.0f;
}