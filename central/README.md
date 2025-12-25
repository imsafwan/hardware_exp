## Central Server for Multi-Agent Planning in Search and Surveillance

This repository contains the central server software for coordinating UAV (Unmanned Aerial Vehicle) and UGV (Unmanned Ground Vehicle) agents in surveillance tasks, particularly for disaster response scenarios. The system integrates reinforcement learning (RL) based planning,  online replanning, and AI-powered image analysis for hazard detection.

### Overview

The central server runs on a ground laptop and handles:
- Communication with UAV and UGV agents via SSH and UDP sockets
- Initial mission planning using RL or heuristic methods
- Online replanning based on agent requests and environmental changes
- Image analysis using Large Language Models (LLMs) for hazard detection
- Coordination of multi-agent actions in discretized search areas (AOIs - Areas of Interest)

### Key Components

#### Main Scripts

1. **`central_check_v5.py`** - Main orchestrator script that:
   - Establishes SSH connections to UAV and UGV
   - Launches agent control scripts via tmux sessions
   - Runs initial mission planning
   - Manages communication during execution
   - Handles AOI status tracking and updates

#### Planning and Solving Scripts

2. **`solver_RL.py`** - Deep Reinforcement Learning policy for joint UAV/UGV action planning
   - Uses trained DRL model for optimal routing and task allocation
   - Integrates with RL_Inference module for inference on current scenarios


3. **`SPRP_uav.py`** / **`SPRP_ugv.py`** - Specialized Replanning for UAV/UGV
   - Handles replanning requests from individual agents
   - Accounts for agent-specific constraints (e.g., UAV battery, UGV terrain)

5. **`replanner1.py`** - Main replanning coordinator
   - Manages online replanning during mission execution
   - Integrates with agent feedback and environmental updates

#### AI and Perception Scripts

6. **`llm_worker.py`** - LLM-powered image analysis for hazard detection
   - Processes images received from UAV
   - Uses GPT-4o-mini to detect hazards (victims, obstacles, smoke/fire)
   - Provides binary classification for decision making

7. **`image_receiver.py`** - Handles image reception from UAV
   - Manages incoming image streams
   - Queues images for LLM analysis

#### Utility Scripts



8. **`scene_discretize.py`** - Discretizes continuous environments
   - Converts geographic coordinates to discrete planning nodes
   - Handles area coverage planning




### RL Inference Module (`RL_Inference/`)

Contains the reinforcement learning components for solving the Multi-agent Cooperative Search and Rescue Problem (MCSRP):

- **`problems/mcsrp/`** - Problem definitions and state management
- **`nets/`** - Neural network architectures (attention models, pointer networks)
- **`model/DRL/`** - Trained DRL models
- **`sampling_evaluation.py`** - Inference and evaluation utilities



### Experiment Workflow

1. **Preparation**:
   - Configure scenario in `Inputs/scene.yaml`
   - Ensure UAV and UGV control scripts are ready (`uav_task.sh`, `ugv_task.sh`)

2. **Launch Central Server**:
   ```bash
   cd src
   python3 central_check_v5.py
   ```

3. **Execution Flow**:
   - Server connects to agents and starts their control systems
   - Runs initial planning (RL or heuristic)
   - Sends action plans to agents
   - Monitors execution and handles replanning requests
   - Processes images and updates AOI status
   - Continues until mission completion

4. **Monitoring**:
   - Check logs in `logs/` directory
   - View agent actions in `output/` directory
   - Monitor AOI status in `aoi_status.txt`

### Outputs

- **Action Plans**: YAML files in `output/` (uav_actions.yaml, ugv_actions.yaml)
- **Logs**: Timestamped log files in `logs/`
- **Images**: Received images in `received_images/`




