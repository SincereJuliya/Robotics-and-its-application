# Robot Planning Project

This project implements a ROS2-based robot task and motion planning system, including path planning with obstacle avoidance, victim rescue prioritization, and trajectory validation using Dubins paths.

## To Run Instructions

  - **to start all nodes:**
    ./to_run_nodes.sh

  - **to stop all nodes:**
    ./kill_nodes.sh

-- Change the paths inside the scripts if processes still run

---

## Nodes

### 1. MapGeneratorNode

- Generates a collision-free navigation graph of the robot's environment.
- Supports two map generation strategies:
  - **Sample-Based Map Generation**
  - **Cell-Decomposition Map Generation** (default)
- Subscribes to:
  - `/shelfino/amcl_pose` — Robot's start pose
  - `/gates` — Positions of gates
  - `/borders` — Map boundaries
  - `/obstacles` — Obstacles as polygon arrays
- Publishes the generated graph on `/generated_graph`.

### 2. TaskPlannerNode

- Plans optimal paths considering victims, obstacles, and time constraints.
- Uses an A* Greedy algorithm with victim prioritization and penalties.
- Subscribes to:
  - `/generated_graph` — The navigation graph
  - `/shelfino/amcl_pose` — Start pose
  - `/gates`, `/borders`, `/obstacles`, `/victims` — Environment data
  - `/victims_timeout` — Time limit for victim rescue
- Uses the `validate_path` service to verify generated paths with the motion planner.
- Handles failed and slow path segments by applying penalties in planning.

### 3. MotionPlannerNode

- Validates planned paths against robot motion constraints using Dubins paths.
- Subscribes to environmental data and communicates with path validation services.
- Publishes motion trajectories for execution.

### 4. FollowPath

- Helps follow the generated paths.

---
