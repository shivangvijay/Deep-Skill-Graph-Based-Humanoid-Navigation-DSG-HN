# Deep Skill Graph Based Humanoid Navigation (DSG-HN)

## Installation

### 1. Create and activate a Conda environment

```bash
conda create -n dsg-hn python=3.10 -y
conda activate dsg-hn
```

### 2. Clone the repository

```bash
git clone git@github.com:shivangvijay/Deep-Skill-Graph-Based-Humanoid-Navigation-DSG-HN.git
cd Deep-Skill-Graph-Based-Humanoid-Navigation-DSG-HN
```

### 3. Install dependencies and build

Install system dependencies:

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

Install **MuJoCo** (for C++ simulator). See the [unitree_mujoco installation instructions](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation) (C++ simulator) for details. In short:

1. Download the [MuJoCo release](https://github.com/google-deepmind/mujoco/releases) and extract it to `~/.mujoco`.
2. From the repo root, create a symlink so the simulator can find MuJoCo:

```bash
cd unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
mkdir build && cd build
cmake ..
make -j
cd ../../..
```

Install **unitree_sdk2**:

```bash
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF   # Installs to /usr/local
sudo make install
cd ../..
```

Build the **g1_29dof** robot:

```bash
cd DSG_HN/deploy/robots/g1_29dof
mkdir build && cd build
cmake .. && make
cd ../../..
```

After this, the installation is complete and you can run the project from the repository root.

---

## Starting the application

### 1. Connect the Xbox joystick

Plug in your Xbox controller before starting the simulator.

### 2. Recommended: activate the Conda environment in all terminals

In each terminal you use, run:

```bash
conda activate dsg-hn
```

### 3. Run the simulator and controller

**Terminal 1 — MuJoCo simulator:**

```bash
cd unitree_mujoco/simulate/build
./unitree_mujoco
```

**Terminal 2 — G1 controller:**

```bash
cd DSG_HN/deploy/robots/g1_29dof/build
./g1_ctrl --network lo
```

### 4. Joystick and keyboard setup

Use the joystick and keyboard in this order:

1. Press **[L2 + Up]** to set the robot to stand up.
2. Click the MuJoCo window, then press **8** to make the robot feet touch the ground.
3. Press **[R1 + X]** to run the policy.
4. Click the MuJoCo window, then press **9** to disable the elastic band.

After that, you can use the Xbox controller to move the robot.

### 5. Using Robot Bridge

#### 5.1 Setup
```bash
cd DSG_HN/deploy/dsg
mkdir build && cd build
cmake..
make -j$(nproc)
```

To run the example program:
```bash
./build_dsg --network lo
```

#### 5.2 Using RobotBridge

The RobotBridge class, defined in dsg/robot_bridge.h, can be used to abstract away the dds communication required to read state information from the mujoco simulation, and send velocity commands. The key functions are as follows:

1. **RobotBridge(string network, string scene_filepath, x_min, x_max, y_min, y_max)** constructor, where the network should be the same as the low level controller (see example file for how to set it up), the filepath should point to the .xml file (again, see the example for how this is done), and the x/y min and max are the maximum bounds at which you want to spawn the robot.
2. **resetRobot()** resets the robot to a new, safe starting location.**It must be called before sending any velocity commands.**
3. **publishVelCommand(vector\<float\> cmd)** sends a velocity command to the low level policy, which actuates the robots joints. The vector inputs consists of the velocity in the x and y directions, as well as the anglular yaw velocity.
4. **getRobotState()** returns the state as type RobotState. See the program for the exact fields, but holds information on positon, velocity, angular velocity, etc.



---

## Cheatsheet

### 1. Cyclone DDS CLI (Python package)

Use the same **domain and network** as your apps (e.g. domain 0, interface `lo`):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cyclonedds ls
```
```bash
# List DDS entities (participants, readers, writers) and QoS
cyclonedds ls

# List applications/participants
cyclonedds ps

# Show type(s) used for a topic (if type discovery works)
cyclonedds typeof rt/lowstate

# Subscribe and print data as it arrives (may show raw/hex for custom Unitree types)
cyclonedds subscribe rt/lowstate
```
