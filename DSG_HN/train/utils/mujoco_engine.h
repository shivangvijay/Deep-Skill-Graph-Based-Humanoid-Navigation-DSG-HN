#pragma once
#include <mujoco/mujoco.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <GLFW/glfw3.h>

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

    // 1. Initialize: Loads the XML and allocates MuJoCo structures
    void Initialize(const std::string &xml_path);

    // 2. Step: Advances the simulation by exactly one timestep (m->opt.timestep)
    void Step();

    // 3. Data Access: Returns pointers to the raw MuJoCo objects
    mjModel *GetModel() const { return m; }
    mjData *GetData() const { return d; }

    // Helper to get the current simulation time
    double GetTime() const { return d ? d->time : 0.0; }

    // Helper to set control inputs (torques/targets)
    void SetControl(const double *ctrl);

    bool render;
    void Render();
    bool IsWindowOpen() const;

private:
    void InitViz();

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