#include "mujoco_utils/mujoco_engine.h"
#include <iostream>
#define Z_START_HEIGHT 0.9

MuJoCoEngine::MuJoCoEngine(bool render_) : render_m(render_) {}

void MuJoCoEngine::initialize(const std::string &xml_path)
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

    if (render_m)
    {
        initViz();
    }
}

void MuJoCoEngine::initViz()
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

void MuJoCoEngine::render()
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

bool MuJoCoEngine::isWindowOpen() const
{
    return window && !glfwWindowShouldClose(window);
}

void MuJoCoEngine::step()
{
    if (m && d)
    {
        mj_step(m, d);
    }
}

// Gonna have to modify this, cannot set control directly based on action, since action gives joint pos
void MuJoCoEngine::setControl(const double *ctrl)
{
    // TODO: Might want to add control noise
    if (d && m)
    {
        mju_copy(d->ctrl, ctrl, m->nu);
    }
}

void MuJoCoEngine::reset(const std::array<float, 3> &pos, const std::array<float, 4> &quat)
{
    mj_resetData(m, d);
    d->qpos[0] = pos[0];
    d->qpos[1] = pos[1];
    d->qpos[2] = Z_START_HEIGHT;
    d->qpos[3] = quat[0];
    d->qpos[4] = quat[1];
    d->qpos[5] = quat[2];
    d->qpos[6] = quat[3];
    mj_forward(m, d);
}

bool MuJoCoEngine::inCollision()
{
    if (!d || !m)
        return false;

    for (int i = 0; i < d->ncon; i++)
    {
        mjContact *contact = &d->contact[i];

        int geom1 = contact->geom1;
        int geom2 = contact->geom2;

        std::string name1 = mj_id2name(m, mjOBJ_GEOM, geom1) ? mj_id2name(m, mjOBJ_GEOM, geom1) : "";
        std::string name2 = mj_id2name(m, mjOBJ_GEOM, geom2) ? mj_id2name(m, mjOBJ_GEOM, geom2) : "";

        if (name1 != "floor" && name1 != "ground" && 
            name2 != "floor" && name2 != "ground") 
        {
            return true;
        }
    }
    return false;
}