# Behavior Tree Synthesis with LLMs

A research project to advance robot programming through large language models (LLMs) by generating context-sensitive Behavior Trees (BTs) for ROS2 environments.

## Overview
This project develops a prototype for intuitive robot programming, integrating six LLM-driven modules: **ChatBT** for conversational BT editing, **Multi-Modal Prompting** for vision-language BT synthesis, **Preference Memory** for personalized BTs, **Semantic Task Decomposition** for high-level command translation, **Real-Time Error Recovery** for robust execution, and **Collaborative Multi-Robot Coordination** for synchronized multi-robot tasks. Deployable in ROS2, it supports tasks like navigation and manipulation, enhancing usability and reliability.

## Features
- **Conversational Interface**: Edit and debug BTs via natural language (ChatBT).
- **Context-Aware Synthesis**: Generate BTs from text and visual inputs (Multi-Modal Prompting).
- **Personalization**: Adapt BTs to user preferences (Preference Memory).
- **Task Decomposition**: Translate abstract commands into executable BTs.
- **Error Recovery**: Detect and resolve runtime failures with explanations.
- **Multi-Robot Support**: Coordinate multiple robots for collaborative tasks.
- **Evaluation**: Validated via Gazebo simulations and TurtleBot3 testing.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/behavior-tree-synthesis.git
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
4. Run the prototype:
   ```bash
   ros2 run bt_synthesis main
   ```

## Dataset
A comprehensive dataset of natural language commands, visual inputs, and BTs will be available in `/data` upon project completion.

## Contributing
Contributions are welcome! Please submit issues or pull requests for bug fixes, enhancements, or new features.

## License
MIT License

## Acknowledgments
Developed for CSE465 at North South University under Dr. Mohammad Ashrafuzzaman Khan. Targeting publications at ICRA, IROS, and HRI.