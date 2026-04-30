# Transformer-Based Transition Model & MPC Controller

This directory contains the **Transformer transition model** for the Unitree G1 humanoid and a **Model Predictive Controller (MPC)** that uses the learned model to drive the robot toward goals in MuJoCo simulation.

## Overview

### What we did

1. **Trained a Transformer transition model** (`transition_train_transformer_delta.cpp`) that predicts *state deltas* (change in position, orientation, velocity, and joint states) given a history of (state, action) tokens. This replaces the earlier MLP-based transition model with a sequence model that can leverage temporal context.

2. **Built an offline rollout evaluator** (`transition_rollout_eval_transformer_delta.cpp`) to validate the model's multi-step prediction accuracy by unrolling trajectories from recorded data and comparing against ground truth.

3. **Created a real-time MPC controller** (`sandbox_mpc`) that:
   - Loads a trained Transformer checkpoint and normaliser statistics.
   - Connects to the MuJoCo-simulated G1 robot via `robot_bridge`.
   - At each control step, samples 256 random action sequences over a 10-step horizon, rolls them out through the Transformer, and executes the first action from the best (lowest-cost) trajectory.
   - Supports keyboard/joystick input for resetting the robot and setting new goals interactively.

### Architecture

The Transformer transition model uses:
- **Token projection** from raw (state + action) vectors to `d_model`-dimensional embeddings.
- **Learned positional embeddings** for the sequence.
- **Causal self-attention** (decoder-style) so each token can only attend to itself and prior tokens.
- **Feed-forward blocks** with SiLU activation.
- **Gaussian output heads** (mean + log-variance) for probabilistic predictions of state deltas.

Default hyperparameters: `d_model=128`, `n_heads=4`, `n_layers=4`, `history=10`.

### File structure

| File | Purpose |
|------|---------|
| `transition_train_transformer_delta.cpp` | Train the Transformer transition model on collected trajectory data |
| `transition_rollout_eval_transformer_delta.cpp` | Evaluate multi-step rollout accuracy offline |
| `sandbox_mpc.cpp` | Main loop: robot bridge, MuJoCo sim, keyboard/joystick, goal tracking (no torch headers) |
| `sandbox_mpc_torch.cpp` | Transformer model definition, normaliser, MPC planner (torch-only TU) |
| `sandbox_mpc_torch.h` | Shared interface between the two sandbox_mpc translation units |

> **Why two translation units for sandbox_mpc?**
> System-installed `spdlog` links against `fmt` v9, while `libtorch` bundles `fmt` v11+. Including both in the same compilation unit causes symbol conflicts. Splitting into `sandbox_mpc.cpp` (spdlog/robot_bridge side) and `sandbox_mpc_torch.cpp` (libtorch side) with a clean C++ API between them avoids the clash entirely.

## Prerequisites

- **libtorch** (PyTorch C++ API) — place under `DSG_HN/train/thirdparty/libtorch/` or update `LIBTORCH_PATH` in `CMakeLists.txt`.
- **MuJoCo** — via the `unitree_mujoco` submodule.
- **Conda environment** (`dsg-hn`) with `libsvm` (only needed for `train`/`test` targets, not sandbox_mpc).
- **System packages**: `libboost-program-options-dev`, `libyaml-cpp-dev`, `libspdlog-dev`, `libfmt-dev`, `libeigen3-dev`, `libglfw3-dev`.
- **CUDA toolkit** (optional, for GPU inference).

## Build

```bash
conda activate dsg-hn
cd DSG_HN/train
mkdir -p build && cd build

cmake .. \
  -DCMAKE_C_COMPILER=gcc-11 \
  -DCMAKE_CXX_COMPILER=g++-11 \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11

make sandbox_mpc -j$(nproc)
```

## Training the Transformer Model

First collect trajectory data (via the `train` executable), then:

```bash
cd DSG_HN/train/build

./transition_train_transformer_delta \
  --data_dir <path-to-trajectory-csvs> \
  --output_dir output_transformer_delta \
  --epochs 500 \
  --batch_size 64 \
  --lr 1e-4 \
  --history 10 \
  --d_model 128 \
  --n_heads 4 \
  --n_layers 4
```

This saves:
- `output_transformer_delta/transition_transformer_delta_best.pt` — best checkpoint
- `output_transformer_delta/normaliser.txt` — input/action/target normalisation statistics

## Running the MPC Controller

```bash
cd DSG_HN/train/build

./sandbox_mpc \
  --checkpoint output_transformer_delta/transition_transformer_delta_best.pt \
  --normaliser output_transformer_delta/normaliser.txt \
  --goal-x 3.0 \
  --goal-y 0.0 \
  --history 10 \
  --d-model 128 \
  --n-heads 4 \
  --n-layers 4
```

If `--normaliser` is omitted, it defaults to `normaliser.txt` in the same directory as the checkpoint.

### Controls (while running)

| Key / Button | Action |
|--------------|--------|
| `G` | Enter a new goal (x, y) via stdin |
| `R` / Joystick button 6 | Reset robot to random position |
| `0` / Joystick button 7 / Ctrl+C | Quit |

### MPC Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MPC_CANDIDATES` | 256 | Number of random action sequences sampled per step |
| `MPC_HORIZON` | 10 | Rollout depth (steps into the future) |
| `CONTROL_HZ` | 10 | Control frequency (Hz) |
| `GOAL_REACHED_DIST` | 0.15 | Distance threshold to declare goal reached (m) |

These are compile-time constants in `sandbox_mpc.cpp` and `sandbox_mpc_torch.cpp`.

## Evaluating Rollout Accuracy

```bash
cd DSG_HN/train/build

./transition_rollout_eval_transformer_delta \
  --checkpoint output_transformer_delta/transition_transformer_delta_best.pt \
  --data_dir <path-to-trajectory-csvs> \
  --history 10 \
  --d_model 128 \
  --n_heads 4 \
  --n_layers 4
```

## Notes

- The **MLP-based transition model** and its MPC (`transition_mpc_eval`) live on the `transition-mpc` branch and are separate from the Transformer-based code here.
- Action space: `vx` in [-0.5, 1.0], `vy` in [-0.3, 0.3], `yaw_rate` in [-1.0, 1.0].
- State vector: quaternion orientation (4) + linear velocity (2) + angular velocity z (1) + joint positions (35) + joint velocities (35) = 77 dims.
- The model predicts **deltas** (changes), not absolute next states.
