# PROWL-R

**Patrolling Robotic Observer with Wartime Logic & Response**

A multi-agent Capture the Flag simulation built in [Webots R2025a](https://cyberbotics.com/) using e-puck robots. Two teams of autonomous agents compete to capture the opposing team's flag and return it to their base.

---

## Overview

| | Team A | Team B |
|---|---|---|
| **Spawn** | Left half (x < 0) | Right half (x > 0) |
| **Enemy flag** | Green (right base) | Yellow (left base) |
| **Comms channel** | 1 | 2 |

Each team has one **attacker** and one **defender** (defenders in progress). All robots run independent behaviour trees with no centralised controller.

---

## Repository Structure

```
ProgrammingForRoboticsFinalAssignmentGC1/
├── worlds/
│   └── PROWL-R_V1.wbt              # Webots world file
├── controllers/
│   ├── attacker_controller/
│   │   └── attacker_controller.py  # Attacker BT (5 states)
│   └── defender_controller/
│       └── defender_controller.py  # Defender BT (5 states)
└── README.md
```

---

## Requirements

- [Webots R2025a](https://cyberbotics.com/#download)
- Python 3.x (bundled with Webots)
- No external dependencies

---

## Getting Started

1. Clone the repository
2. Open Webots and load `worlds/PROWL-R_V1.wbt`
3. Press **Play** — both attacker robots start automatically

### VSCode setup (optional)

Add the following to `.vscode/settings.json` to resolve the `controller` import:

```json
{
  "python.analysis.extraPaths": [
    "C:\\Program Files\\Webots\\lib\\controller\\python"
  ],
  "python.defaultInterpreterPath": "C:\\Program Files\\Webots\\msys64\\mingw64\\bin\\python3.exe"
}
```

---

## Behaviour Trees

Both controllers use a **priority-based behaviour tree** — the highest-priority condition that is true wins each timestep.

### Attacker

| Priority | State | Condition |
|---|---|---|
| 1 | `AVOID_OBSTACLE` | Any front-arc proximity sensor > 80 |
| 2 | `RECOVERY` | Stuck > 3 s or avoidance loop > 3 s |
| 3 | `RETURN_TO_BASE` | Flag captured |
| 4 | `EVADE_DEFENDER` | Defender visible in camera *(stub)* |
| 5 | `SEEK_FLAG` | Default |

### Defender

| Priority | State | Condition |
|---|---|---|
| 1 | `AVOID_OBSTACLE` | Any front-arc proximity sensor > 80 |
| 2 | `RECOVERY` | Stuck > 3 s or avoidance loop > 3 s |
| 3 | `RETURN_TO_POST` | Intruder tagged or lost |
| 4 | `CHASE_INTRUDER` | Intruder visible in camera *(stub)* |
| 5 | `PATROL_ZONE` | Default |

---

## Key Systems

- **Odometry** — wheel encoder integration for (x, y, θ) dead-reckoning; re-zeroed at known waypoints to bound drift
- **Navigation** — proportional heading-error steering with [-π, π] wrap
- **Perception** — pixel-by-pixel RGB camera scan to detect flag colours (green/yellow) and opponents
- **Obstacle avoidance** — sensor-weighted turn direction, escape counter to prevent corridor oscillation, two-phase recovery (reverse → turn)
- **Communication** — Emitter/Receiver broadcast of position, heading, time, and flag status each timestep; per-team channels; solo mode on 5 s timeout

---

## Team

Oscar Chia, Blair Fenton, Lakshay

*Programming for Robotics — Option C: Multi-Agent Coordination*
