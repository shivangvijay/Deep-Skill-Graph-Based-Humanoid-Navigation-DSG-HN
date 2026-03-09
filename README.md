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
cd ../..
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

### 1. Connect the Unitree joystick (or gamepad)

Use the **Unitree wireless joystick** if you have it—plug its receiver into the PC so it appears as a joystick device (e.g. `/dev/input/js0`). If you don't have it, you can use an **Xbox/PC gamepad** instead; the simulator will treat it as an emulated Unitree wireless controller. Connect before starting the simulator.

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

Use the Unitree joystick (or gamepad) and keyboard in this order:

1. Press **[L2 + Up]** to set the robot to stand up.
2. Click the MuJoCo window, then press **8** to make the robot feet touch the ground.
3. Press **[R1 + X]** to run the policy.
4. Click the MuJoCo window, then press **9** to disable the elastic band.

After that, you can use the Unitree joystick or gamepad to move the robot.

---

## Cheatsheet

### 1. Check network interface

Find your Ethernet interface name (for robot connection):

```bash
ip link show
# or
ls /sys/class/net/
```

Use the wired Ethernet interface (e.g. `eno1`, `eth0`) for the robot; `lo` for simulation on the same PC.

### 2. Cyclone DDS CLI

Use the same **domain and network** as your apps. For simulation (loopback):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cyclonedds ps
```

For real robot (Ethernet, e.g. eno1):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cyclonedds ps
```

Replace `eno1` with your interface from `ip link show`. Other commands:

```bash
cyclonedds ls
cyclonedds ps
cyclonedds typeof rt/lowstate
cyclonedds subscribe rt/lowstate
```

### 3. Sim2Real setup

No simulator is started for Sim2Real; only `g1_ctrl` runs and talks to the robot over the network.

1. Check network interface with `ip link show` or `ls /sys/class/net/` (e.g. `eno1`).
2. Set DDS to use Ethernet: `export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'`
3. Update config (`unitree_mujoco/simulate/config.yaml`): set `use_joystick: 0` when using the Unitree wireless controller.
4. Put G1 in debug mode (when suspended and damping):
   - Press **L2 + R2** on the remote → G1 enters debug mode.
   - Press **L2 + A** → position mode, diagnostic pose.
   - Press **L2 + B** → damping state (confirms debug mode).
5. Verify connection: run `cyclonedds ps` — you should see the robot's DDS participants.
6. Put robot down.
7. Start controller: `./g1_ctrl --network eno1` (replace `eno1` with your interface).

### 4. Shutdown process

See [Unitree G1 Developer Quick Start](https://support.unitree.com/home/en/G1_developer/quick_start) for the proper shutdown procedure.
