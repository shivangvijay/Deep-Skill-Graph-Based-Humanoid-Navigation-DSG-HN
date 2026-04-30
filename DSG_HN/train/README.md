# Transition Model Training & MPC Pipeline

End-to-end pipeline for learning a dynamics model of the Unitree G1 humanoid robot and using it for Model Predictive Control (MPC) navigation in MuJoCo.

## Overview

```
Data Collection  -->  Training  -->  Evaluation  -->  MPC
(joystick/sim)       (Transformer    (rollout        (CEM planner +
                      or MLP)        eval)           live MuJoCo)
```

Three model variants are provided:

| Model | Architecture | Checkpoint Dir |
|---|---|---|
| Transformer Baseline | 4-layer causal Transformer, no multi-step loss | `checkpoints/baseline/` |
| Transformer Improved | Same arch + multi-step loss, tuned weights, cosine LR | `checkpoints/improved/` |
| MLP Gaussian Delta | 3-layer MLP (256→256→128) | `checkpoints/mlp/` |

## Prerequisites

- **Conda environment**: `dsg-hn` with libtorch (PyTorch C++ API), Boost, yaml-cpp
- **MuJoCo** installed via `unitree_mujoco/simulate/mujoco/`
- **Joystick** (optional) for manual data collection; keyboard fallback available

## Directory Structure

```
DSG_HN/train/
├── sandbox_mpc.cpp                          # MPC main loop (robot + MuJoCo rendering)
├── sandbox_mpc_torch.cpp                    # Batched CEM planner + Transformer inference
├── sandbox_mpc_torch.h                      # MPC API header
├── transition_train_transformer_delta.cpp   # Transformer trainer (toggleable improvements)
├── transition_train_gaussian_delta.cpp      # MLP trainer
├── transition_rollout_eval_transformer_delta.cpp  # Transformer rollout evaluator
├── transition_rollout_eval_gaussian_delta.cpp     # MLP rollout evaluator
├── CMakeLists.txt
├── checkpoints/
│   ├── baseline/       # Transformer without improvements
│   ├── improved/       # Transformer with multi-step loss, tuned weights, cosine LR
│   └── mlp/            # MLP Gaussian Delta model
└── README.md

DSG_HN/data_collection/
├── collect_sim_transitions.cpp   # Data collection script
└── transitions_umaze.csv         # 32K transitions collected on umaze_scene
```

## Step 1: Build Everything

```bash
conda activate dsg-hn

# Build the training executables
cd DSG_HN/train/build
cmake .. && make -j$(nproc)

# Build the data collector (if you need to collect more data)
cd ../../transition_model/build
cmake .. && make collect_sim_transitions -j$(nproc)
```

This builds:
- `transition_train_transformer_delta` — Transformer trainer
- `transition_train_gaussian_delta` — MLP trainer
- `transition_rollout_eval_transformer_delta` — Transformer rollout evaluator
- `transition_rollout_eval_gaussian_delta` — MLP rollout evaluator
- `sandbox_mpc` — Live MPC with MuJoCo rendering
- `collect_sim_transitions` — Joystick/keyboard data collector

## Step 2: Data Collection

The provided `transitions_umaze.csv` contains ~32K transitions collected on `umaze_scene.xml`. To collect more data:

```bash
cd DSG_HN/transition_model/build
./collect_sim_transitions
```

**Controls:**
- **Left joystick**: forward/backward (vx) and strafe (vy)
- **Right joystick**: yaw rotation
- **Keyboard fallback**: W/S (vx), A/D (vy), Q/E (yaw)
- **R** or joystick button 6: reset robot to random position
- **0** or joystick button 7: stop and save

Data is appended to `transitions_umaze.csv` in the build directory. The CSV format:

```
timestamp_s, x, y, z, qw, qx, qy, qz, vx, vy, vz, omega_x, omega_y, omega_z,
joint_pos_00..34, joint_vel_00..34, cmd_vx, cmd_vy, cmd_yaw,
next_x, next_y, ..., next_joint_vel_34
```

Each row is one transition: `(state, action) -> next_state`.

## Step 3: Train the Transformer Model

All three improvement features are now CLI-toggleable:

### Baseline (no improvements)

```bash
cd DSG_HN/train/build

./transition_train_transformer_delta \
  --csv ../../../DSG_HN/data_collection/transitions_umaze.csv \
  --output-dir ./output_baseline \
  --epochs 1000 \
  --batch-size 64 \
  --lr 1e-4 \
  --history 10 \
  --d-model 128 \
  --n-heads 4 \
  --n-layers 4 \
  --future-steps 0 \
  --multistep-weight 0 \
  --seed 42
```

### Improved (all features ON)

```bash
./transition_train_transformer_delta \
  --csv ../../../DSG_HN/data_collection/transitions_umaze.csv \
  --output-dir ./output_improved \
  --epochs 1000 \
  --batch-size 64 \
  --lr 1e-4 \
  --history 10 \
  --d-model 128 \
  --n-heads 4 \
  --n-layers 4 \
  --future-steps 3 \
  --multistep-weight 0.3 \
  --tuned-weights \
  --cosine-lr \
  --seed 42
```

### Improvement Flags

| Flag | Default | Effect |
|---|---|---|
| `--tuned-weights` | OFF | Weighted loss: dx/dy=5, dyaw=3, dvx/dvy=1.5, joints=0.3/0.1 |
| `--cosine-lr` | OFF | Cosine annealing LR with 5% linear warmup |
| `--future-steps K` | 3 | Multi-step rollout loss horizon (set 0 to disable) |
| `--multistep-weight W` | 0.3 | Weight for multi-step loss (annealed from 0 to W over 30% of training) |

## Step 4: Train the MLP Model

```bash
./transition_train_gaussian_delta \
  --csv ../../../DSG_HN/data_collection/transitions_umaze.csv \
  --output-dir ./output_mlp \
  --epochs 1000 \
  --batch-size 64 \
  --lr 1e-3 \
  --patience 100 \
  --dropout 0.2 \
  --seed 42
```

## Step 5: Evaluate Models (Rollout Errors)

The rollout evaluator measures prediction errors in physical units (meters, radians) over multi-step horizons:

```bash
# Evaluate baseline Transformer
./transition_rollout_eval_transformer_delta \
  --csv ../../../DSG_HN/data_collection/transitions_umaze.csv \
  --checkpoint output_baseline/transition_transformer_delta_best.pt \
  --normaliser output_baseline/normaliser.txt \
  --horizons "0.5,1,2,5" \
  --history 10 --d-model 128 --n-heads 4 --n-layers 4

# Evaluate improved Transformer
./transition_rollout_eval_transformer_delta \
  --csv ../../../DSG_HN/data_collection/transitions_umaze.csv \
  --checkpoint output_improved/transition_transformer_delta_latest.pt \
  --normaliser output_improved/normaliser.txt \
  --horizons "0.5,1,2,5" \
  --history 10 --d-model 128 --n-heads 4 --n-layers 4

# Evaluate MLP Gaussian Delta
./transition_rollout_eval_gaussian_delta \
  --csv ../../../DSG_HN/data_collection/transitions_umaze.csv \
  --checkpoint output_mlp/transition_gaussian_delta_model_best.pt \
  --normaliser output_mlp/normaliser.txt \
  --horizons "0.5,1,2,5"
```

**Output metrics** (per horizon):
- **XY error** (m) — position accuracy (most important for MPC)
- **Vel error** (m/s) — velocity prediction
- **Heading error** (rad) — yaw accuracy
- **Joint pos/vel RMSE** — joint state accuracy

## Step 6: Run MPC with Live MuJoCo Rendering

```bash
./sandbox_mpc \
  --checkpoint output_baseline/transition_transformer_delta_best.pt \
  --history 10 --d-model 128 --n-heads 4 --n-layers 4 \
  --goal-x 3.0 --goal-y 0.0
```

Or use the pre-trained checkpoints:

```bash
# With baseline checkpoint
./sandbox_mpc \
  --checkpoint ../checkpoints/baseline/transition_transformer_delta_best.pt \
  --history 10 --d-model 128 --n-heads 4 --n-layers 4 \
  --goal-x 3.0 --goal-y 0.0

# With improved checkpoint
./sandbox_mpc \
  --checkpoint ../checkpoints/improved/transition_transformer_delta_latest.pt \
  --history 10 --d-model 128 --n-heads 4 --n-layers 4 \
  --goal-x 3.0 --goal-y 0.0
```

### MPC Controls

| Key | Action |
|---|---|
| **G** | Enter new goal coordinates |
| **R** | Reset robot to random position |
| **0** / Ctrl+C | Quit |

### MPC Tuning Flags

```bash
--candidates 256       # Number of action sequences sampled (default: 256)
--cem-rounds 3         # CEM refinement rounds (1 = random shooting, default: 3)
--cem-elites 32        # Top candidates used to refit distribution (default: 32)
--horizon 10           # Planning horizon in steps (default: 10, = 1s at 10Hz)
--max-steps 200        # Max steps before timeout stall (default: 200)
--w-pos 1.0            # Per-step distance cost weight
--w-heading 0.5        # Heading alignment cost weight
--w-terminal 3.0       # Terminal position cost weight
--w-smooth 0.1         # Action smoothness cost weight
--w-backward 0.3       # Backward velocity penalty weight
```

The MPC uses **batched CEM**: all candidates are evaluated in a single GPU forward pass per horizon step, making it ~100x faster than sequential evaluation.

## Model Architecture

### Transformer (d=128, h=4, L=4, seq=10)

```
Input: 10 timesteps × 80-D tokens (77-D state + 3-D action)
  → Linear projection to 128-D + positional embedding
  → 4× TransformerBlock (causal self-attention + FFN with SiLU)
  → LayerNorm → mu_head (76-D) + logvar_head (76-D)
Output: predicted state delta (dx, dy, dyaw, dvx, dvy, doz, 35 joint_pos deltas, 35 joint_vel deltas)
```

### MLP Gaussian Delta (256→256→128)

```
Input: 80-D (77-D state + 3-D action) — single timestep
  → Linear(80, 256) + LayerNorm + SiLU + Dropout
  → Linear(256, 256) + LayerNorm + SiLU + Dropout
  → Linear(256, 128) + LayerNorm + SiLU + Dropout
  → mu_head (76-D) + logvar_head (76-D)
Output: same delta prediction as Transformer
```

## Training Results (32K transitions, umaze_scene)

| Model | Best Val Loss | Test Loss | Notes |
|---|---|---|---|
| Transformer Baseline | -1.7 | -1.5 | Uniform weights, flat LR, no multi-step |
| Transformer Improved | -0.7 | -0.6 | Loss values not directly comparable (different weightings) |

**Important**: Val/test loss values between baseline and improved are NOT directly comparable because tuned output weights change the loss scale. Use the rollout evaluator (Step 5) for true comparison — it reports errors in physical units.
