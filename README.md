# Guide Robot using ROS-LLM and Nav2 Packages

Welcome to the repository for DTETI UGM's guide robot development, which integrates ROS-LLM and Nav2 packages. This repository is forked from the original [Auromix/ROS-LLM](https://github.com/Auromix/ROS-LLM).

To explore detailed information about:
- **ROS-LLM**: Visit the [original repository](https://github.com/Auromix/ROS-LLM).
- **Nav2 integration**: Refer to the [Nav2 ROS Wiki](https://navigation.ros.org/).

---

## ✨ Quickstart Guide

### 1. Fork and Clone the Repository
First, fork this repository into your own GitHub account. Then, clone it to your local machine using the command below:

```bash
git clone https://github.com/your_github_account/llm-guide-robot.git
```

Ensure you place the **llm_config** package from the original ROS-LLM repository into your local workspace (e.g., `llm-guide-robot`).

### 2. Install Dependencies
Navigate to the `llm_install` directory and execute the installation script to install the required dependencies:

```bash
cd llm-guide-robot/llm_install
bash dependencies_install.sh
```

### 3. Configure OpenAI Settings
Obtain an OpenAI API key from the [OpenAI Platform](https://platform.openai.com) if you don’t already have one. Then, configure the API key by running the script:

```bash
cd llm-guide-robot/llm_install
bash config_openai_api_key.sh
```

### 4. Configure OpenAI Whisper Settings (Optional)
To enable local natural interaction capabilities, configure the OpenAI Whisper settings. For low-performance edge devices, cloud-based ASR services are recommended to reduce computing overhead. For high-performance systems, local ASR services can be used for faster responses.

Install Whisper and related dependencies:

```bash
pip install -U openai-whisper
pip install setuptools-rust
```

### 5. Build the Workspace
Navigate to your workspace directory and build the ROS2 workspace:

```bash
cd llm-guide-robot
rosdep install --from-paths src --ignore-src -r -y  # Install dependencies
colcon build --symlink-install
```

### 6. Run the Demo
Source the workspace and launch the Turtlesim demo:

```bash
source llm-guide-robot/install/setup.bash
ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py
```

#### Interact with the Demo
Start listening:
```bash
ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 'listening'" -1
```

Provide feedback:
```bash
ros2 topic pub /llm_state_feedback std_msgs/msg/String "data: 'feedback text'" -1
```

---

## ⚙️ Using the Framework with Nav2

To integrate the framework with your own robot, modify the `llm_robot` and `llm_config` packages to match your robot’s specifications. This customization allows you to define the behavior and control mechanisms for your robot.

### Steps to Test with Nav2 System:
1. Source the workspace:
   ```bash
   source llm-guide-robot/install/setup.bash
   ```

2. Launch the Nav2 system:
   ```bash
   ros2 launch navigation tb3_maze1.launch.py
   ```

3. Alternatively, run another Nav2 configuration:
   ```bash
   ros2 launch navigation navigation.launch.py
   ```

4. Configure RViz:
   - Set up proper topics/messages.
   - Estimate the robot’s position.
   - Send the goal in the map to test navigation.

---

### Additional Notes
- Ensure all dependencies are correctly installed and sourced before launching any package.
- For troubleshooting or further details, refer to the original ROS-LLM and Nav2 documentation.

---

Happy exploring!

