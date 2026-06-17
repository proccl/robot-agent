# Robot-Agent

MATLAB-based 7-DOF robot arm visualization with natural language control via file-system queue.

## Architecture

```
User (Kimi CLI)  --Natural Language-->  AI generates .m script  --File Queue-->  MATLAB  --Animation-->  Figure
```

- **robotagent.m**: One-click launcher (file watch + animation engine)
- **Arm7R.m**: 7-DOF forward/inverse kinematics
- **quinticTrajectory.m**: Quintic polynomial joint-space trajectory planner
- **processIncomingCommands.m**: File-queue executor (`timer` callback)
- **Pure MATLAB**, no Robotics Toolbox required, no Python bridge, no TCP server

> **v0.0.5 update**: The old intermediate layers `generate_robot_cmd.m` and `parseNaturalLanguage.m` have been removed. Kimi now generates executable `.m` scripts directly and drops them into `incoming/`.

## Quick Start

### Step 1: Launch (MATLAB GUI)

In MATLAB, navigate to the `robot-agent` folder and **run** `robotagent.m` (double-click or press ▶):

```
========================================
RobotAgent started.
Figure: RobotAgent - 7R Arm
Watching: D:\...\robot-agent\incoming
========================================
```

A Figure window pops up showing the 7R arm at zero pose. The MATLAB command line remains free.

### Step 2: Send Commands (Kimi CLI)

Type natural language commands in Kimi CLI, e.g.:

```
Move end-effector to (500, 0, 800) in 3 seconds
```

Kimi CLI internally:
1. Reads the project skill `skills/robotagent-ops/SKILL.md`
2. Generates a complete `.m` script (trajectory + animation + log output)
3. Writes it to `robot-agent/incoming/cmd_xxx.m`
4. MATLAB `timer` detects and executes automatically
5. Kimi reads `logs/log_xxx.txt` and reports `[Done]` / `[ERR]` / `[PAUSE]`

## Supported Commands

| Natural Language | Generated Action |
|-----------------|------------------|
| `home` / `回到原位` | Return to zero pose (default 5s) |
| `move to (500, 0, 800)` | Cartesian PTP move (default 5s) |
| `joint 1 90 deg` / `關節1轉90度` | Single-joint move (default 5s) |
| `circle radius 200` / `畫圓 半徑200` | Circular trajectory around end-effector Z axis |
| `status` / `現在姿態` | Print current joint angles & EE pose |
| Composite command | Split into multiple `cmd_*.m` scripts executed in creation order |

Append `in X seconds` / `用 X 秒` to override the default 5-second duration.

## Tests

Run all phases at once:

```matlab
cd('D:\Document\code\Matlab\robot-agent\tests\phases');
addpath('../../src');
run_all_tests;
```

Or run individual phases:

```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests/phases');

test_phase1_figure;
test_phase2_filewatch;
test_phase3_generator;
test_phase4_cleanup;
test_phase5_e2e;
test_phase6_complex;
```

Or run obstacle tests:

```matlab
addpath('src');
addpath('tests/obstacle');

test_build_robot_tree;
test_obstacle_visualization;
test_obstacle_collision;
test_obstacle_avoidance_move;
test_obstacle_disabled;
```

Or run from PowerShell/CMD:

```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent\tests\phases'); addpath('../../src'); run_all_tests;"
```

## Project Structure

```
robot-agent/
├── robotagent.m              # One-click launcher
├── src/
│   ├── Arm7R.m               # 7-DOF kinematics
│   ├── initRobotFigure.m     # Figure initialization (with obstacle sphere)
│   ├── updateRobotFigure.m   # Figure efficient update
│   ├── animateRobot.m        # Animation playback
│   ├── quinticTrajectory.m   # Quintic polynomial planner
│   ├── processIncomingCommands.m # File-queue executor
│   ├── buildRobotTree.m      # Convert Arm7R DH to rigidBodyTree
│   ├── checkRobotObstacleCollision.m # Robot-to-sphere collision check
│   ├── planTrajectoryWithObstacle.m  # RRT-based obstacle avoidance
│   └── computeTrajectory.m   # Legacy trajectory helper
├── tests/
│   ├── phases/
│   │   ├── run_all_tests.m       # Unified test entry
│   │   ├── test_phase1_figure.m
│   │   ├── test_phase2_filewatch.m
│   │   ├── test_phase3_generator.m
│   │   ├── test_phase4_cleanup.m
│   │   ├── test_phase5_e2e.m
│   │   ├── test_phase6_complex.m
│   │   └── output/               # Test screenshots
│   └── obstacle/
│       ├── test_build_robot_tree.m
│       ├── test_obstacle_visualization.m
│       ├── test_obstacle_collision.m
│       ├── test_obstacle_avoidance_move.m
│       └── test_obstacle_disabled.m
├── skills/
│   └── robotagent-ops/       # Project skill for Kimi CLI
│       ├── SKILL.md
│       └── references/
├── docs/
│   ├── README_Arm7R.md
│   ├── plan_refactor_filewatch.md
│   └── robot_agent_cmds.json
└── README.md                 # This file
```

## Changelog

### v0.0.7
- Enhanced obstacle sphere visualization:
  - Added `light` + `lighting gouraud` and material properties to the obstacle sphere
  - Added semi-transparent ground plane
  - Added ground projected shadow for the obstacle sphere
  - Shadow visibility toggles with the obstacle sphere
- Updated project skill (`skills/robotagent-ops/SKILL.md`) with operational conventions:
  - Direction conventions: right = -end-effector X, forward = +end-effector Z, down = -world Z
  - Composite motion sequence templates using joint-space `quinticTrajectory`
  - Handling of fixed-orientation unreachable poses via joint-space interpolation or roll-angle search
- Validated a 7-segment composite motion sequence via natural language control

### v0.0.6
- Added obstacle avoidance module (requires Robotics System Toolbox):
  - Red sphere obstacle visualization in `initRobotFigure.m`
  - `buildRobotTree.m` converts Arm7R DH parameters to `rigidBodyTree`
  - `checkRobotObstacleCollision.m` detects robot-to-sphere collisions
  - `planTrajectoryWithObstacle.m` uses `manipulatorRRT` to plan collision-free paths
  - Manual obstacle avoidance switch: set `enable_obstacle_avoidance = true;` in `robotagent.m` before running (requires Robotics System Toolbox)
- Switches `fig.UserData.obstacle_enabled` and `fig.UserData.obstacle_avoidance_enabled`
- Reorganized `tests/` into `tests/phases/` and `tests/obstacle/`
- Added obstacle avoidance script template G in skill references

### v0.0.5
- Refactored to **AI-direct-code mode**: removed `generate_robot_cmd.m` and `parseNaturalLanguage.m`
- Added unified test entry `tests/run_all_tests.m`
- Reorganized test phases:
  - `test_phase4_cleanup.m`
  - `test_phase5_e2e.m` (end-to-end integration)
  - `test_phase6_complex.m` (composite multi-segment commands)
- Added project skill references for pose adjustment, script templates, and troubleshooting
- Added `incoming_history/` to preserve every executed script
- Improved logging: every execution writes to `logs/log_*.txt` with `[RX]`, `[Done]`, `[ERR]`, or `[PAUSE]` markers

### v0.0.4
- Refactored skill into reference files
- Fixed `processIncomingCommands.m` variable scope bug
- Added circle trajectory templates

### v0.0.3
- Unified logging, pose display
- Composite commands, multi-cmd queue

### v0.0.2
- Added project skill, updated AGENTS.md

### v0.0.1
- File-watch architecture, natural language control, quintic trajectory

## Compatibility

- **MATLAB**: R2020b or later (`timer` required). Tested on R2023b.
- **Toolbox**: None required. Pure base MATLAB.
- **Quintic trajectory**: All joint motions use quintic polynomial interpolation (zero start/end velocity & acceleration).
- **Default duration**: 5 seconds when not specified by user.
