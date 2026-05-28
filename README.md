# Robot-Agent

MATLAB-based 7-DOF robot arm visualization and TCP remote control system.

## Architecture

```
User (Kimi CLI)  --JSON/TCP-->  MATLAB RobotAgent Server  --Animation-->  Figure
```

- **RobotAgent.m**: TCP server + animation engine + trajectory planner
- **Arm7R.m**: 7-DOF forward/inverse kinematics
- Pure MATLAB, no Robotics Toolbox, no Python bridge

## Quick Start

### MATLAB GUI Mode

```matlab
cd('D:\Document\code\Matlab\robot-agent');
run_robot_agent;  % starts server on port 12345
```

### MATLAB Batch Mode

```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent'); addpath('src'); addpath('scripts'); run_robot_agent;"
```

## TCP Commands

Send JSON strings (with LF terminator) to `127.0.0.1:12345`.

| Command | Example | Description |
|---------|---------|-------------|
| `home` | `{"cmd":"home","duration":2}` | Return to zero pose |
| `move_to` | `{"cmd":"move_to","position":[500,0,800],"duration":3}` | Cartesian PTP move |
| `trajectory` | `{"cmd":"trajectory","type":"circle","radius":200,"duration":5}` | Predefined trajectory |
| `set_speed` | `{"cmd":"set_speed","factor":2.0}` | Animation speed factor |
| `get_status` | `{"cmd":"get_status"}` | Query current joint angles & EE pose |
| `plot` | `{"cmd":"plot"}` | Force refresh figure |

See [docs/robot_agent_cmds.json](docs/robot_agent_cmds.json) for full protocol specification.

## Tests

```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests');
test_phase1_figure;
test_phase2_tcp;
test_phase3_render;
test_phase4_compute;
test_phase5_integration;
```

Or run all at once:

```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent'); addpath('src'); addpath('tests'); test_phase1_figure; test_phase2_tcp; test_phase3_render; test_phase4_compute; test_phase5_integration;"
```

## Project Structure

```
robot-agent/
├── src/
│   ├── RobotAgent.m        # Main server class
│   ├── Arm7R.m             # 7-DOF kinematics
│   └── computeTrajectory.m # Trajectory helper
├── scripts/
│   └── run_robot_agent.m   # One-click launcher
├── tests/
│   ├── test_phase1_figure.m
│   ├── test_phase2_tcp.m
│   ├── test_phase3_render.m
│   ├── test_phase4_compute.m
│   ├── test_phase5_integration.m
│   └── output/             # Test screenshots
├── docs/
│   ├── README_Arm7R.md
│   └── robot_agent_cmds.json
└── README.md               # This file
```

## Compatibility

- **MATLAB**: R2020b or later (`tcpserver` required). Tested on R2023b.
- **Toolbox**: None required. Pure base MATLAB.
- **Batch mode limitation**: `parfeval` / `backgroundPool` cannot serialize user-defined functions in `-batch` mode. Trajectory computation falls back to synchronous execution; rendering uses an async `timer` loop.
