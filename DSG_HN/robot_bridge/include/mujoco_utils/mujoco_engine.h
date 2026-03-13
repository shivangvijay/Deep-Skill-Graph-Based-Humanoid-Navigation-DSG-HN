#pragma once
#include <mujoco/mujoco.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <GLFW/glfw3.h>
#include <array>

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
    bool inCollision();
    bool isWindowOpen() const;

private:
    void initViz();

    mjModel *m = nullptr;
    mjData *d = nullptr;

    // rendering
    GLFWwindow *window = nullptr;
    mjvCamera cam;   // abstract camera
    mjvOption opt;   // visualization options
    mjvScene scn;    // abstract scene
    mjrContext con;  // custom GPU context
    mjvPerturb pert; // mouse perturbation
};