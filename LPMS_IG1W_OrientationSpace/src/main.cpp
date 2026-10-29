#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

#include <GLFW/glfw3.h>

//========================================================================
int main()
{
    ofGLFWWindowSettings settings;

    settings.setSize(1280, 720);

    settings.windowMode = OF_WINDOW;
    settings.resizable = false;

    auto window = ofCreateWindow(settings);

    // Remove Windows title bar and borders.
    auto glfwWindow = std::dynamic_pointer_cast<ofAppGLFWWindow>(window);

    if (glfwWindow)
    {
        glfwSetWindowAttrib(glfwWindow->getGLFWWindow(), GLFW_DECORATED, GLFW_FALSE);

        // Ensure position after removing decorations.
        glfwSetWindowPos(glfwWindow->getGLFWWindow(), 1280, 0);
    }

    ofRunApp(window, std::make_shared<ofApp>());
    ofRunMainLoop();
}
