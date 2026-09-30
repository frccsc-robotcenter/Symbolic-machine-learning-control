# AGENTS.md

## What this is

C++14 CMake project implementing a genetic algorithm (GA) that optimizes a **Network Operator (NetOper)** for mobile robot control. The NetOper is a graph-based computation model with a square integer matrix (Psi) and float parameter vector (Cs). The GA evolves both matrix structure and parameters.

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Requires:
- ONNX Runtime at `/opt/onnxruntime` (hardcoded in CMakeLists.txt)
- Boost (`program_options`) — `apt install libboost-program-options-dev`
- Google Test — git submodule at `test/include/googletest` (run `git submodule update --init`)

## Executables

| Binary | Source | Purpose |
|--------|--------|---------|
| `train` | `app/train_robot_control.cpp` | Main GA optimization for robot control |
| `simple_function` | `app/simple_function.cpp` | Simpler GA demo on a toy function |

Run from `build/` — the ONNX model path (`rosbot_gazebo9_2d_model.onnx`) is relative to CWD.

## Tests

```bash
cd build
make nop_tests
ctest
# or directly: ./test/nop_tests
```

Tests use Google Test. Test data (`test/test_data/`) is copied to the build directory automatically via a post-build command.

## Architecture

```
lib/include/         — core headers (NetOper, GA, model, interfaces)
lib/                 — core implementation
app/include/         — problem-specific config (RobotProblemConfig, RobotFitnessEvaluator)
app/                 — executables
data/                — default matrix/params (24x24)
test/                — Google Test suite
```

### Key types

- **`NetOper`** (`lib/include/nop.hpp`) — the network operator. Holds matrix Psi, parameters Cs, node assignments. Methods: `calcResult()`, `loadMatrixFromFile()`, `saveMatrixToFile()`, etc.
- **`GAConfig`** (`lib/include/GAConfig.hpp`) — GA hyperparameters + injected `IFitnessEvaluator` and `ISolution` factory.
- **`GANOP`** (`lib/include/GANOP.hpp`) — the GA engine. Takes `GAConfig`, runs evolution via `run()`.
- **`IFitnessEvaluator`** (`lib/include/ifitness_evaluator.hpp`) — interface for fitness functions.
- **`ISolution`** (`lib/include/isolution.hpp`) — interface for solutions (decode from chromosome, expose NetOper).
- **`RobotProblemConfig`** (`app/include/RobotProblemConfig.hpp`) — robot-specific config inheriting `BaseConfig`. Contains the default 24x24 matrix and trajectory generation logic.
- **`Model`** (`lib/include/model.hpp`) — robot dynamics model using ONNX Runtime inference.

### Design pattern

The GA is generic: `GAConfig` receives an `IFitnessEvaluator` and an `ISolution` factory via dependency injection. To add a new optimization problem, implement these two interfaces and provide a config.

## Matrix size

The active matrix in `RobotProblemConfig` is **24x24** (not 32x32 — the README is outdated). Nodes 0-2 are variables (x, y, theta), nodes 3-10 are parameters (8 total), nodes 22-23 are outputs.

## Data files

- `data/default_matrix.txt`, `data/default_q.txt` — reference matrix and initial state
- `best_matrix.txt`, `best_params.txt` — created after optimization, auto-loaded on next run
- `rosbot_gazebo9_2d_model.onnx` — ONNX model for robot dynamics (must be in CWD when running)

## Python scripts

- `compare.py`, `plot_results_simple.py`, `viz_traj.py` — visualization/analysis utilities (require matplotlib, pandas)

## Conventions

- Code comments and README are in Russian
- C++14 standard, compiled with `-Wall -O3`
- Headers use `#pragma once`
- No package manager — dependencies are system-installed or git submodules
