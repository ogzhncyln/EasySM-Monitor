# EasySM Monitor

A real-time PyQt5-based visualization tool for state machine monitoring in ROS environments. This package provides an intuitive graphical interface to visualize, monitor, and debug state machine execution with enhanced routing, tooltips, and logging capabilities.

## Installation

### Prerequisites

- ROS (tested with ROS Noetic)
- Python 3.6+
- PyQt5

### Install Dependencies

```bash
# Install PyQt5
sudo apt-get install python3-pyqt5

# Or using pip
pip3 install PyQt5
```

### Build the Package

```bash
# Navigate to your catkin workspace
cd ~/catkin_ws

# Clone or copy the package to src/
# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

## Usage

### Basic Launch

```bash
# Launch with default parameters
roslaunch easysm_monitor easysm_monitor.launch

# Launch with custom tree file
roslaunch easysm_monitor easysm_monitor.launch tree_file:=/path/to/your/file.easysm_tree

# Launch with custom monitor topic
roslaunch easysm_monitor easysm_monitor.launch monitor_topic:=/your_monitor_topic
```

### Keyboard Shortcuts
- `Ctrl+S`: Save current state machine configuration

