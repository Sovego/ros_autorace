# ROS2 AutoRace package

A __ROS2__ package for the AutoRace challenge, created in Python.


## Build
__IMPORTANT__: Python version must be greater than or equal to `3.10`.

### Cloning
Clone the repository from github by running the command in the terminal:


`git clone https://github.com/Sovego/ros_autorace.git`.


### Requirements
Unzip the archive, navigate to the directory and install the requirements by running the following command: 


`pip install -r requirements.txt`.

### Package build
Build the package using colcon by running the command in the terminal:


`colcon build --packages-select bagodelnya_core`

### Source the workspace

`. ~/<your ws>/install/setup.bash`

## Run

### Launch the simulation
Run file with the road, traffic signs, obstacles, etc:


`ros2 launch robot_bringup autorace_2023.launch.py`


### Launch the package


`ros2 launch bagodelnya_core bagodelnya.launch.py`
