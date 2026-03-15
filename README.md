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
#### 3.1 System Dependencies
```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```
#### 3.2 Mujoco Installation


Install **MuJoCo** (for C++ simulator). See the [unitree_mujoco installation instructions](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation) (C++ simulator) for details. In short:

1. Download the [MuJoCo release](https://github.com/google-deepmind/mujoco/releases) and extract it to `~/.mujoco`.
2. From the repo root, create a symlink so the simulator can find MuJoCo:

#### 3.2 Libtorch Installation
For training, libtorch is used. Follow the below instructions for installation and setup:
1. Verify that your GPU is cuda enabled
```bash
nvidia-smi
```
2. Install cuda toolkit. **Note that the cuda toolkit should be >= 11.8, and should be compatible with the cuda version installed on your machine.** To install, follow the instructions at: https://developer.nvidia.com/cuda-toolkit-archive
3. Download the libtorch version compatible with the cuda toolkit. If your cuda toolkit version >= 12.6 or you are installing for CPU, can find at https://pytorch.org/get-started/locally/. If you have an older version, can find by using the template: download.pytorch.org/libtorch/cu<version_number\>, and select the top link for download. For example, for CUDA 11.8 navigate to https://download.pytorch.org/libtorch/cu118.
4. Verify installation:
```bash
nvcc --version
```
4. Extract the directory, and copy the underlying folder, named libtorch, into DSG_HN/train/thirdparty

#### 3.3 Build Libraries


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

Build **robot_bridge** examples:

For **deployment**:
```bash
cd DSG_HN/deploy/src
mkdir build && cd build
make -j$(nproc)
```

For **training**: \
Before proceeding, identify the version of gcc that the NVCC (cuda compiler) you have is compatible with. Check with the command
```bash
nvcc --version
```
Note that CUDA 12.1+ supports GCC 12, but CUDA 11 requires GCC11

Install the training examples
```bash
cd DSG_HN/train
mkdir build && cd build
cmake ..   -DCMAKE_C_COMPILER=<GCC version>   -DCMAKE_CXX_COMPILER=<G++ version>   -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/<cuda version> -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/<GCC version>
make -j$(nproc)
```

As an example cmake command, for CUDA 11.8, and therefore GCC and G++ 11, use:
```bash
cmake ..   -DCMAKE_C_COMPILER=gcc-11   -DCMAKE_CXX_COMPILER=g++-11   -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8   -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11
```

Note that some of the flags are required due to not setting system paths, and having multiple versions of nvcc on my machine, so the entirety of this may not be necessary.

---

## Starting the application

### Training

See DSG_HN/train for example training and testing files. Additionally, this repository contains an example gym environment and TD3 implementation that has been used to train a waypoint follower with 95%+ accuracy. To run the example training script, first navigate to build and uncomment the obstacles in the ai_maker_space_scene.xml file to simplify the env. Then run:

```bash
cd DSG_HN/train/build
./train
```

To run the testing script:
```bash
./test
```

### Deployment

#### 1. (If using a Xbox Joystick) Connect the Xbox joystick

Plug in your Xbox controller before starting the simulator.

#### 2. Recommended: activate the Conda environment in all terminals

In each terminal you use, run:

```bash
conda activate dsg-hn
```

#### 3. Run the simulator

In a new terminal:

```bash
cd unitree_mujoco/simulate/build
./unitree_mujoco
```

#### 4. Run low level controller

Three high level control modes are supported: Robot Bridge DDS, Xbox joystick, and keyboard.

1. Identify which method you want to use to control the robot. 
2. Navigate to `deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml`
3. At line 43, uncomment one of the following options:
   - **Robot Bridge (deploy or training)**: dsg_velocity_commands
   - **Keyboard**: keyboard_velocity_commands
   - **Controller**: velocity_command
4. Start the low level controller in a new terminal:
   ```bash
   cd DSG_HN/deploy/robots/g1_29dof/build
   ./g1_ctrl --network lo
   ``` 
#### 5. Send commands to the robot

**Robot Bridge DDS**: /

Follow example.cpp to see how to setup robot bridge via dds.

```bash
cd DSG_HN/deploy/src/build
./example --network lo
```

If the elastic band is enabled in mujoco (which you can disable in unitree_mujoco/simulate/config.yaml), press **`8`** until the feet touch the ground, and then press **`9`** to disable.

**Keyboard**:
In the **g1_ctrl terminal**, use these keys in order:

1. Press **`1`** → robot stands up (FixStand mode).
2. Click the **MuJoCo window**, then press **`8`** → feet touch the ground.
3. Back in the **g1_ctrl terminal**, press **`2`** → policy starts running (Velocity mode).
4. Click the **MuJoCo window**, then press **`9`** → disable the elastic band.
5. Back in the **g1_ctrl terminal**, use **`w/s/a/d/q/e`** to move the robot:
   - `w` / `s` — forward / backward
   - `a` / `d` — strafe left / right
   - `q` / `e` — rotate left / right
   - `0` — return to Passive mode

**Xbox joystick**
Plug in your Xbox controller before starting the simulator. Use the joystick in this order:

1. Press **[L2 + Up]** to set the robot to stand up.
2. Click the MuJoCo window, then press **8** to make the robot feet touch the ground.
3. Press **[R1 + X]** to run the policy.
4. Click the MuJoCo window, then press **9** to disable the elastic band.

After that, use the Xbox controller to move the robot.

### 5. RobotBridge Overview

The RobotBridge class, defined in DSG/robot_bridge/include/robot_bridge.h, can be used to abstract away the setup and communication with the robot, both for deplyment (RobotBridgeDDS), and training (RobotBridgeTrain). See the examples provided in DSG_HN/deploy/src and DSG_HN/train for more information on their usage.

Overall, RobotBridge aims to provide an interface such that at both training and deployment time, the same commands/algorithms can be used without modification to control the robot. Additionally, it aims to simplify the development of RL algorithms by providing an abstraction layer.

For more information on the RobotBridge class and its functionality, see DSG/robot_bridge/

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
