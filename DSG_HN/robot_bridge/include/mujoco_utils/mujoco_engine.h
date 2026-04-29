#pragma once
#include <mujoco/mujoco.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <GLFW/glfw3.h>
#include <array>

void silent_warning_handler(const char *msg);
void mouse_button(GLFWwindow *window, int button, int action, int mods);
void mouse_move(GLFWwindow *window, double xpos, double ypos);
void scroll(GLFWwindow *window, double xoffset, double yoffset);

class MuJoCoEngine
{
public:
    MuJoCoEngine(bool render);

    ~MuJoCoEngine()
    {
        if (window)
        {
            mjr_freeContext(&con);
            mjv_freeScene(&scn);
            glfwDestroyWindow(window);
            glfwTerminate();
            window = nullptr;
        }
        if (d)
            mj_deleteData(d);
        if (m)
            mj_deleteModel(m);
    }

    void initialize(const std::string &xml_path);

    void step();

    mjModel *getModel() const { return m; }
    mjData *getData() const { return d; }

    double getTime() const { return d ? d->time : 0.0; }

    void setControl(const double *ctrl);

    bool render_m;
    void render();
    void reset(const std::array<float, 3> &pos, const std::array<float, 4> &quat);
    void reset(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel);
    bool inCollision();
    bool isWindowOpen() const;

    void addMarker(const mjtNum pos[3], float radius,
                   const float rgba[4], mjtGeom type = mjGEOM_SPHERE);
    void clearMarkers();

    double lastx = 0;
    double lasty = 0;
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;

    mjModel *m = nullptr;
    mjData *d = nullptr;

    // rendering
    GLFWwindow *window = nullptr;
    mjvCamera cam;   // abstract camera
    mjvOption opt;   // visualization options
    mjvScene scn;    // abstract scene
    mjrContext con;  // custom GPU context
    mjvPerturb pert; // mouse perturbation

    struct Marker {
        mjtNum pos[3];
        mjtNum size[3];
        float rgba[4];
        mjtGeom type;
    };
    std::vector<Marker> markers_;

    void initViz();

    void setGoalMarker(float x, float y, float z)
    {
        goal_marker_pos[0] = x;
        goal_marker_pos[1] = y;
        goal_marker_pos[2] = z;
        goal_marker_active = true;
    }

    void clearGoalMarker() { goal_marker_active = false; }

    struct ObstacleMarker { float x, y, radius; };
    void setObstacleMarkers(const std::vector<ObstacleMarker> &markers, float margin)
    {
        obstacle_markers = markers;
        obstacle_margin = margin;
    }

    float goal_marker_pos[3] = {0, 0, 0};
    bool goal_marker_active = false;
    std::vector<ObstacleMarker> obstacle_markers;
    float obstacle_margin = 0.8f;
};