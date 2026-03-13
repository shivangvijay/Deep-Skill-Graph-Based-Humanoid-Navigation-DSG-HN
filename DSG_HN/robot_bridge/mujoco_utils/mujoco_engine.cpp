#include "mujoco_engine.h"
#include <iostream>

MuJoCoEngine::MuJoCoEngine(bool render_) : render(render_) {}

void MuJoCoEngine::Initialize(const std::string &xml_path)
{
    char error[1000];

    // Load XML and compile into mjModel
    m = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
    if (!m)
    {
        std::cerr << "Could not load model: " << error << std::endl;
        return;
    }

    // Make data structure
    d = mj_makeData(m);

    // Run one forward pass to initialize all fields (positions, etc.)
    mj_forward(m, d);

    std::cout << "MuJoCo initialized with model: " << xml_path << std::endl;

    if (render)
    {
        InitViz();
    }
}

void MuJoCoEngine::InitViz()
{
    // Initialize GLFW
    if (!glfwInit())
        return;

    window = glfwCreateWindow(1200, 900, "MuJoCo Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize MuJoCo visualization objects
    mjv_defaultCamera(&cam);
    cam.elevation = -90.0;
    cam.azimuth = 90.0;
    cam.distance = 10.0;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.0;
    
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
}

void MuJoCoEngine::Render()
{
    if (!window)
        return;

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

bool MuJoCoEngine::IsWindowOpen() const
{
    return window && !glfwWindowShouldClose(window);
}

void MuJoCoEngine::Step()
{
    if (m && d)
    {
        mj_step(m, d);
    }
}

// Gonna have to modify this, cannot set control directly based on action, since action gives joint pos
void MuJoCoEngine::SetControl(const double *ctrl)
{
    if (d && m)
    {
        mju_copy(d->ctrl, ctrl, m->nu);
    }
}