# ENPM-661-Project-3-A_Star-Phase 2

This repository contains the code for Project 3 Phase 2 of the ENPM 661 course. The project focuses on implementing the A* algorithm for path planning in a given environment.

## Installation

To use this code, you need to have Python installed on your system. You can download Python from the official website: [Python.org](https://www.python.org/).

## Dependencies
- Python 3.7
- OpenCV
- Numpy
- Queue
- Time
- Rclpy
- Rclpy.node
- geometry_msgs

## Usage

1. Clone the repository to your local machine:

    ```bash
    git clone https://github.com/Nitish05/ENPM661-Project-3-Phase-2.git
    ```

2. Navigate to the project directory:

    ```bash
    cd ENPM-661-Project-3-Phase-2
    ```

3. Run the main script:

    ```bash
    python planner.py
    ```

4. To run the script in Gazebo Mode, you need to have ROS2 installed on your system. If you have ROS2 installed and followed the instructions [here](https://github.com/shantanuparabumd/turtlebot3_project3) (the same link as in the documentation), you can run the following command to launch the robot in gazebo:

    ```bash
    ros2 launch turtlebot3_project3 competition_world.launch.py
    ```
    Then to run script run the following command:

    ```bash
    ros2 run turtlebot3_project3 planner.py
    ```
 
5.  
    - The scrpit will ask the user to choose a mode (1 for Gazebo Mode, 2 for 2D Mode).
    - Case 1:
        - If the user selects Gazebo Mode and ROS2 libraries are installed, the script will prompt the user to enter the goal positions. The script will then generate the optimal path using the A* algorithm and display the path.
        - If the ROS2 libraries are not installed, the script will display a message indicating that Gazebo Mode will not function, and the script will continue to run in 2D Mode.
        - The path is displayed in a popup window.
        - Assuming the user has already launched the robot in gazebo.
        - Closing the popup window will start publishing the linear velocities and angular velocities to the turtlebot3 robot in gazebo.
        - Note:
            - The default start position is (50, 95, 0)
            - RPM values for the left and right wheels are set to 10, 20
            - Clearance distance is set to 2
            ```bash
            _____________________________________
                __          __                   
                / |       /    )                 
            ---/__|-------\------_/_----__---)__-
              /   |        \     /    /   ) /   )
            _/____|____(____/___(_ __(___(_/_____
                                                

            Enter the mode (1 for Gazebo Mode, 2 for 2D Mode): 1    

            Gazebo mode selected.


            Generating the map...


            The goal node should be within the canvas dimensions (25-575, 25-175) and not inside an obstacle.

            Enter the goal node X: 575
            Enter the goal node Y: 175
            Enter the goal node Angle: 0
            Start Node:  (50, 95, 0)
            Goal Node:  (575, 175, 0)

            Calculating the path...
            Goal Reached
            Cost to Goal:  659.1763334230152
            Execution time: 31.7377 seconds
            [INFO] [1712536727.823658395] [velocity_publisher]: Publishing: "geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.017278759594743863, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-0.1204094745278318))"
            [INFO] [1712536727.924740196] [velocity_publisher]: Publishing: "geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.017278759594743863, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-0.1204094745278318))"
            .
            .
            .
            ```
    - Case 2:
        - If the user selects 2D Mode, the script will prompt the user to enter the clearance distance, robot radius, RPM values for the left and right wheels, start and goal positions.
        - The script will then generate the optimal path using the A* algorithm and display the path.
        - The path is displayed in a popup window.
        - The script will also display the cost to the goal and the execution time.
        ```bash
            _____________________________________
                __          __
                / |       /    )
            ---/__|-------\------_/_----__---)__-
              /   |        \     /    /   ) /   )
            _/____|____(____/___(_ __(___(_/_____


            Enter the mode (1 for Gazebo Mode, 2 for 2D Mode): 1
            ROS2 libraries are not installed. Gazebo mode will not function.

            2D mode selected.

            Clearance distance should be a positive number
            Enter the clearance distance: 2
            Robot radius should be a positive number
            Enter the robot radius: 22

            Generating the map...

            Enter the RPM values for the left and right wheels eg. 10, 20
            Enter the RPM 1: 10
            Enter the RPM 2: 20

            The start node and goal node should be within the canvas dimensions (25-575, 25-175) and not inside an obstacle.

            Enter the start node X: 50
            Enter the start node Y: 100
            Enter the start node Angle: 0
            Enter the goal node X: 575
            Enter the goal node Y: 100
            Enter the goal node Angle: 0
            Start Node:  (50, 100, 0)
            Goal Node:  (575, 100, 0)

            Calculating the path...
            Goal Reached
            Cost to Goal:  611.641843668552
            Execution time: 27.5063 seconds
        ```
6. The script will also save a video of the path generated. The video will be saved in the same directory as the script.
        
## Contributors
Nitish Ravisankar Raveendran - rrnitish - 120385506\
Pranav ANV - anvpran - 1204886110

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

## Acknowledgements

Special thanks to the ENPM 661 course instructors for providing the project requirements and guidance.

## Video Links
- [2D Mode](https://youtu.be/HQzprgOGBaI) - starting from (25, 25, 0) to (573, 173, 0) clearance 2, radius 22, RPM (10, 25)
- [Gazebo Mode](https://youtu.be/dQbWYm2Mwug) - starting from (50, 95, 0) to (575, 175, 0)


## References
https://github.com/Nitish05/ENPM-661-Project-3-A_Star
