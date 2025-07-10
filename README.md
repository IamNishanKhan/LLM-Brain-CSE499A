# Behavior Tree Synthesis with LLMs - CSE499A

A research project for CSE499A at North South University to advance robot programming through large language models (LLMs) by generating context-sensitive Behavior Trees (BTs) for ROS2 environments.

## Overview
This project, developed for CSE499A at North South University, aims to create a prototype for intuitive and adaptive robot programming. It integrates six LLM-driven modules: **ChatBT** for conversational BT editing and debugging, **Multi-Modal Prompting** for generating BTs from text and visual inputs, **Preference Memory** for user-personalized BT synthesis, **Semantic Task Decomposition** for translating high-level commands into executable BTs, **Real-Time Error Recovery** for robust runtime failure handling, and **Collaborative Multi-Robot Coordination** for synchronized multi-robot task execution. Deployable in ROS2, the system targets complex tasks like indoor navigation, object manipulation, and multi-robot collaboration, enhancing usability, personalization, and reliability for both novice and expert users in academic and industrial applications.

## Features
- **Conversational Interface**: Enables natural language-based BT editing and debugging (ChatBT).
- **Context-Aware Synthesis**: Generates BTs using combined language and visual inputs (Multi-Modal Prompting).
- **Personalized BTs**: Adapts BT generation to user preferences (Preference Memory).
- **Task Decomposition**: Converts abstract commands into structured BTs (Semantic Task Decomposition).
- **Error Recovery**: Detects and resolves runtime errors with natural language explanations (Real-Time Error Recovery).
- **Multi-Robot Coordination**: Supports synchronized task execution across multiple robots (Collaborative Multi-Robot Coordination).
- **Evaluation Plan**: Validation through Gazebo simulations and real-world testing on platforms like TurtleBot3.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/IamNishanKhan/LLM-Brain-CSE499A.git
   ```
2. Install dependencies:
   - ROS2 Humble
   - BehaviorTree.CPP
   - Gazebo
   - Python 3.8+ with `transformers`, `torch`, `opencv-python`
3. Set up ROS2 workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build
   ```
4. Run the prototype (once implemented):
   ```bash
   ros2 run bt_synthesis main
   ```

## Contributing
Contributions are welcome! Please submit issues or pull requests for bug fixes, enhancements, or new features.

## License
MIT License

## Acknowledgments
Developed for CSE499A at North South University under the supervision of Dr. Mohammad Ashrafuzzaman Khan.