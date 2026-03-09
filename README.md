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

## Sim2Sim and Sim2Real

### Prerequisites: unitree_mujoco and config

1. Install **unitree_mujoco** (see Installation and MuJoCo steps above).
2. In **`unitree_mujoco/simulate/config.yaml`** set:
   - `robot: "g1"`
   - `domain_id: 0`
   - `enable_elastic_band: 1`
   - `use_joystick: 1` (Unitree joystick or gamepad)

### Sim2Sim (simulation only)

**Terminal 1 — start simulation:**

```bash
cd unitree_mujoco/simulate/build
./unitree_mujoco
# Alternative (explicit robot, scene, domain, interface):
# ./unitree_mujoco -i 0 -n lo -r g1 -s scene_29dof.xml
```

**Terminal 2 — G1 controller (use loopback for sim):**

```bash
cd DSG_HN/deploy/robots/g1_29dof/build
./g1_ctrl --network lo
```

Then in order:

1. Press **[L2 + Up]** to stand the robot up.
2. Click the MuJoCo window and press **8** so the feet touch the ground.
3. Press **[R1 + X]** to run the policy.
4. Click the MuJoCo window and press **9** to disable the elastic band.

You can then drive the robot with the Unitree joystick or gamepad.

### Sim2Real (control the physical robot)

Use the same controller to drive the **real G1**.

1. **Put the robot into debug / developer mode** (or the mode that accepts external low-level control) so that your PC can send motor commands. Typically this means the built-in on-board control is not actively driving the robot—check Unitree's G1 documentation or app for the exact menu name and steps (e.g. "Developer mode", "Sport mode", or "Wireless controller").
2. Run the controller from your PC:

```bash
cd DSG_HN/deploy/robots/g1_29dof/build
./g1_ctrl --network eth0   # Replace eth0 with your Ethernet interface (e.g. eno1)
```

No simulator is started for Sim2Real; only `g1_ctrl` runs and talks to the robot over the network.

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

### 2. Cyclone DDS CLI — inspect DDS topics

Set the network interface to match your setup, then run `cyclonedds ls`:

**For simulation (loopback):**

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cyclonedds ls
```

**For real robot (Ethernet, e.g. eno1):**

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cyclonedds ls
```

Replace `eno1` with your interface from `ip link show` (e.g. `eth0`, `enp6s0`).

Other useful commands:

```bash
cyclonedds ps          # List participants
cyclonedds typeof rt/lowstate
cyclonedds subscribe rt/lowstate
```

### 3. Sim2Real setup (Unitree controller, Ethernet)

1. **Check network interface** with `ip link show` or `ls /sys/class/net/` (e.g. `eno1` for Ethernet).
2. **Set DDS to use Ethernet:**

   ```bash
   export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
   ```

3. **Update config** (`unitree_mujoco/simulate/config.yaml`): set `use_joystick: 0` when using the Unitree wireless controller (it talks to the robot directly).
4. **Put G1 in debug mode** (when suspended and damping):
   - Press **L2 + R2** on the remote → G1 enters debug mode.
   - Press **L2 + A** → position mode, diagnostic pose.
   - Press **L2 + B** → damping state (confirms debug mode; useful for hardware check).
5. **Verify connection:** run `cyclonedds ls` — you should see the robot's DDS participants.
6. Put robot down
7. **Start controller:**

   ```bash
   ./g1_ctrl --network eno1
   ```

   Replace `eno1` with your Ethernet interface.

### 4. Shutdown process

See [Unitree G1 Developer Quick Start](https://support.unitree.com/home/en/G1_developer/quick_start) for the proper shutdown procedure.
