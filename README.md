# Robot-Agent

MATLAB-based 7-DOF robot arm visualization with natural language control via file-system queue.

## Architecture

```
User (Kimi CLI)  --Natural Language-->  .m Code  --File Queue-->  MATLAB  --Animation-->  Figure
```

- **robotagent.m**: One-click launcher (file watch + animation engine)
- **Arm7R.m**: 7-DOF forward/inverse kinematics
- **quinticTrajectory.m**: Quintic polynomial joint-space trajectory planner
- **generate_robot_cmd.m**: Code generator (cmd_struct → executable .m file)
- **parseNaturalLanguage.m**: Natural language parser
- Pure MATLAB, no Robotics Toolbox, no Python bridge, no TCP server

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
1. Parses to `cmd_struct`
2. Generates a `.m` script with `quinticTrajectory`
3. Writes it to `robot-agent/incoming/cmd_xxx.m`
4. MATLAB `timer` detects and executes automatically

## Supported Commands

| Natural Language | Generated Action |
|-----------------|------------------|
| `home` / `回到原位` | Return to zero pose (default 5s) |
| `move to (500, 0, 800)` | Cartesian PTP move (Quintic, default 5s) |
| `joint 1 90 deg` | Single-joint move (default 5s) |
| `circle radius 200` | Circular trajectory (default 5s) |
| `status` / `現在姿態` | Print current joint angles & EE pose |

Append `in X seconds` / `用 X 秒` to override the default 5-second duration.

## Tests

```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests');

test_phase1_figure;
test_phase2_filewatch;
test_phase3_generator;
test_phase4_nlp;
test_phase5_cleanup;
test_phase6_integration;
```

Or run all at once:

```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent'); addpath('src'); addpath('tests'); test_phase1_figure; test_phase2_filewatch; test_phase3_generator; test_phase4_nlp; test_phase5_cleanup; test_phase6_integration;"
```

## Project Structure

```
robot-agent/
├── robotagent.m              # One-click launcher
├── src/
│   ├── Arm7R.m               # 7-DOF kinematics
│   ├── initRobotFigure.m     # Figure initialization
│   ├── updateRobotFigure.m   # Figure efficient update
│   ├── animateRobot.m        # Animation playback
│   ├── quinticTrajectory.m   # Quintic polynomial planner
│   ├── generate_robot_cmd.m  # Code generator
│   ├── parseNaturalLanguage.m # NLP parser
│   └── processIncomingCommands.m # File watch executor
├── tests/
│   ├── test_phase1_figure.m
│   ├── test_phase2_filewatch.m
│   ├── test_phase3_generator.m
│   ├── test_phase4_nlp.m
│   ├── test_phase5_cleanup.m
│   ├── test_phase6_integration.m
│   └── output/               # Test screenshots
├── docs/
│   ├── README_Arm7R.md
│   ├── plan_refactor_filewatch.md
│   └── robot_agent_cmds.json
└── README.md                 # This file
```

## Compatibility

- **MATLAB**: R2020b or later (`timer` required). Tested on R2023b.
- **Toolbox**: None required. Pure base MATLAB.
- **Quintic trajectory**: All joint motions use quintic polynomial interpolation (zero start/end velocity & acceleration).
- **Default duration**: 5 seconds when not specified by user.
