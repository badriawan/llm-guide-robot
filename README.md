
# Guide Robot using ROS-LLM and Nav2 Packages
This is the repo for DTETI guide robot development.
This repo forked from the original Auromix/ROS-LLM

If you want to learn the details about ROS-LLM , please check the original repo.

If you want to learn and integrate the LLM with Nav2 , please check the nav2 ros wiki.


## 🔥 Quickstart Guide

Follow the instructions below to set up this packages:

**1. Fork and Clone the Repository:**

Firstly, don't forget to fork this repo into your own repo.

Use the command below to clone the repository.
```bash
git clone https://github.com/your_github_account/llm-guide-robot.git
```
Put the llm_config packages from the original ROS-LLM into your local workspace (llm-guide-robot).

**2. Install Dependencies:**

Navigate to the `llm_install` directory and execute the installation script.
```bash
cd llm-guide-robot/llm_install
bash dependencies_install.sh
```

**3. Configure OpenAI Settings:**

If you don't have an OpenAI API key, you can obtain one from [OpenAI Platform](https://platform.openai.com). Use the script below to configure your OpenAI API key.
```bash
cd llm-guide-robot/llm_install
bash config_openai_api_key.sh
```

**4. Configure OpenAI Whisper Settings (Optional):**

For local natural interaction capabilities, configure the OpenAI Whisper settings. If you prefer to use cloud ASR, this step can be skipped.

For low-performance edge embedded platforms, it is recommended to use ASR cloud services to reduce computing pressure, and for high-performance personal hosts, it is recommended to use local ASR services to speed up response
```bash
pip install -U openai-whisper
pip install setuptools-rust
```

**5. Build the Workspace:**

Navigate to your workspace directory and build the workspace.
```bash
cd llm-guide-robot
rosdep install --from-paths src --ignore-src -r -y  # Install dependencies
colcon build --symlink-install
```

**6. Run the Demo:**

Source the setup script and launch the Turtlesim demo.

```bash
source llm-guide-robot/install/setup.bash
ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py
```
start listening
```bash
ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 'listening'" -1
```
feedback
```bash
ros2 topic pub /llm_state_feedback/
```

## ⚙️ Test it with Nav2 system.

To use the framework with your own robot, modify the `llm_robot` and `llm_config` packages to suit your robot's specifications. This allows you to customize the behavior of your robot.

```bash
source llm-guide-robot/install/setup.bash
ros2 launch navigation tb3_maze1.launch.py
```

```bash
source llm-guide-robot/install/setup.bash
ros2 launch navigation navigation.launch.py
```
Config the Rviz with proper topic/massages and estimate the  position of the robot and send the goal in the map.