# Line-Following Robot

This is a project that utilizes ROS 2 Humble, OpenCV, and Gazebo Fortress Ignition to simulate a vehicle that autonomously follows a black line.

## Installation

1. Create a directory for the workspace
    ```sh
    mkdir ros2_humble_ws
    ```
2. Change directory to the workspace
    ```sh
    cd ros2_humble_ws
    ```
3. Create src directory
    ```sh
    mkdir src
    ```
4. Change directory to src
    ```sh
    cd src
    ```
5. Clone this project
    ```sh
    git clone https://github.com/Howard-GitHub/line-following-robot.git
    ```
6. Change directory back to the root of the workspacce
    ```sh
    cd ..
    ```

7. Run colcon build in ros2_humble_ws
    ```sh
    colcon build
    ```

8. Source setup.bash
    ```sh
    source install/setup.bash
    ```

## Usage

Here are the instructions to run the simulation of the line following robot.

1. Startup the Gazebo application with the vehicle and black line loaded
    ```sh
    ros2 launch line_following_robot gz_spawn.launch.py
    ```
2. Run the line-following program
    ```sh
    ros2 run line_following_robot line_following_node
    ```

## Third-Party Asset

### Gazebo world model by KroNton

Source: https://app.gazebosim.org/KroNton/fuel/models/track1 

License: Creative Commons Attribution 4.0 International

Changes: Integrated into a custom simulation world
