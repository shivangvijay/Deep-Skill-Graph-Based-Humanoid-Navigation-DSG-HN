#include "mujoco_utils/mujoco_engine.h"
#include <iostream>
#define Z_START_HEIGHT 0.9

void silent_warning_handler(const char *msg)
{
    // Do nothing, effectively silencing warnings
}

void mouse_button(GLFWwindow* window, int button, int action, int mods) {
    auto* eng = (MuJoCoEngine*)glfwGetWindowUserPointer(window);
    eng->button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    eng->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    eng->button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(window, &eng->lastx, &eng->lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    auto* eng = (MuJoCoEngine*)glfwGetWindowUserPointer(window);
    double dx = xpos - eng->lastx;
    double dy = ypos - eng->lasty;
    eng->lastx = xpos; eng->lasty = ypos;

    if (!eng->button_left && !eng->button_right && !eng->button_middle) return;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    mjtMouse action = eng->button_right ? mjMOUSE_MOVE_V : 
                     (eng->button_left ? mjMOUSE_ROTATE_V : mjMOUSE_ZOOM);
    
    mjv_moveCamera(eng->m, action, dx/height, dy/height, &eng->scn, &eng->cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    auto* eng = (MuJoCoEngine*)glfwGetWindowUserPointer(window);
    mjv_moveCamera(eng->m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &eng->scn, &eng->cam);
}

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

    mju_user_warning = silent_warning_handler;
}

void MuJoCoEngine::initViz()
{
    // Initialize GLFW
    if (!glfwInit())
        return;

    window = glfwCreateWindow(1200, 900, "MuJoCo Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);


    glfwSetWindowUserPointer(window, this);

    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // Initialize MuJoCo visualization objects
    mjv_defaultCamera(&cam);
    cam.elevation = -90.0;
    cam.azimuth = 90.0;
    cam.distance = 20.0;
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

void MuJoCoEngine::addMarker(const mjtNum pos[3], float radius,
                             const float rgba[4], mjtGeom type)
{
    Marker mk;
    mk.pos[0] = pos[0]; mk.pos[1] = pos[1]; mk.pos[2] = pos[2];
    if (type == mjGEOM_CYLINDER) {
        mk.size[0] = (mjtNum)radius;
        mk.size[1] = 0.02;     // half-height (thin disc)
        mk.size[2] = 0.0;
    } else {
        mk.size[0] = (mjtNum)radius;
        mk.size[1] = (mjtNum)radius;
        mk.size[2] = (mjtNum)radius;
    }
    mk.rgba[0] = rgba[0]; mk.rgba[1] = rgba[1]; mk.rgba[2] = rgba[2]; mk.rgba[3] = rgba[3];
    mk.type = type;
    markers_.push_back(mk);
}

void MuJoCoEngine::clearMarkers()
{
    markers_.clear();
}

void MuJoCoEngine::render()
{
    if (!window)
        return;

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    if (goal_marker_active && scn.ngeom < scn.maxgeom)
    {
        mjvGeom *g = &scn.geoms[scn.ngeom];
        mjtNum pos[3] = {(mjtNum)goal_marker_pos[0], (mjtNum)goal_marker_pos[1], (mjtNum)goal_marker_pos[2]};
        mjtNum size[3] = {0.15, 0.15, 0.15};
        float rgba[4] = {1.0f, 0.0f, 0.0f, 0.8f};
        mjv_initGeom(g, mjGEOM_SPHERE, size, pos, NULL, rgba);
        scn.ngeom++;
    }

    for (const auto &obs : obstacle_markers)
    {
        if (scn.ngeom >= scn.maxgeom) break;
        mjvGeom *g = &scn.geoms[scn.ngeom];
        mjtNum pos[3] = {(mjtNum)obs.x, (mjtNum)obs.y, 0.02};
        float total_r = obs.radius + obstacle_margin;
        mjtNum size[3] = {(mjtNum)total_r, (mjtNum)total_r, 0.01};
        float rgba[4] = {1.0f, 1.0f, 0.0f, 0.3f};
        mjv_initGeom(g, mjGEOM_CYLINDER, size, pos, NULL, rgba);
        scn.ngeom++;
    }

    for (const auto &s : debug_spheres)
    {
        if (scn.ngeom >= scn.maxgeom)
            break;
        mjvGeom *g = &scn.geoms[scn.ngeom];
        mjtNum pos[3] = {(mjtNum)s.x, (mjtNum)s.y, (mjtNum)s.z};
        mjtNum size[3] = {(mjtNum)s.radius, (mjtNum)s.radius, (mjtNum)s.radius};
        float rgba[4] = {s.rgba[0], s.rgba[1], s.rgba[2], s.rgba[3]};
        mjv_initGeom(g, mjGEOM_SPHERE, size, pos, NULL, rgba);
        scn.ngeom++;
    }

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

void MuJoCoEngine::reset(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
{
    mj_resetData(m, d);
    d->qpos[0] = pos[0];
    d->qpos[1] = pos[1];
    d->qpos[2] = Z_START_HEIGHT;
    d->qpos[3] = quat[0];
    d->qpos[4] = quat[1];
    d->qpos[5] = quat[2];
    d->qpos[6] = quat[3];

    d->qvel[0] = vel[0];
    d->qvel[1] = vel[1];
    d->qvel[2] = vel[2];

    d->qvel[3] = ang_vel[0]; // roll
    d->qvel[4] = ang_vel[1]; // pitch
    d->qvel[5] = ang_vel[2];

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

        // want to only ignore collisions with the floor/ground on the
        if (name1 != "floor" && name1 != "ground" &&
            name2 != "floor" && name2 != "ground")
        {
            return true;
        }
    }
    return false;
}
