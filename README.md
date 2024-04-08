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
    cd ENPM-661-Project-3-A_Star
    ```

3. Run the main script:

    ```bash
    python a_star_nitish_pranav.py
    ```

4. Follow the on-screen instructions to input the start and goal positions, and the program will generate the optimal path using the A* algorithm.\
    eg:
    ```bash
    _____________________________________
        __          __
        / |       /    )
    ---/__|-------\------_/_----__---)__-
      /   |        \     /    /   ) /   )
    _/____|____(____/___(_ __(___(_/_____


    Step size should be between 1 and 10
    Enter the step size: 10
    Clearance distance should be a positive number
    Enter the clearance distance: 5
    Robot radius should be a positive number
    Enter the robot radius: 5

    Generating the map...

    The start node and goal node should be within the canvas dimensions (11-1190, 11-490) and not inside an obstacle.

    Enter the start node X: 11
    Enter the start node Y: 11
    Enter the start node Angle: 0
    Enter the goal node X: 1000
    Enter the goal node Y: 250
    Enter the goal node Angle: 180
    Start Node:  (11, 11, 0)
    Goal Node:  (1000, 250, 180)

    Running A* algorithm...
    Goal Reached
    Cost to Goal:  1310
    Execution time: 13.1537 seconds
    ```
5.  
    - The script will prompt the user to enter the step size, clearance distance, and robot radius.
    - The script will take a few seconds to generate the map.
    - The script will prompt the user to enter the start and goal coordinates as well as the orientation in the start and goal node. The origin (0,0) is at the bottom left corner of the grid space.
    - The orientation is such that 0 degrees is along the positive y-axis, 90 degrees is along the positive x-axis, 180 degrees is along the negative y-axis, and 270 degrees is along the negative x-axis. (Like a clock face or compass directions.)
    - The orientation should in 30 degrees increments. (0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330 ...)
    - If the orientation is not in 30 degrees increments, the script will choose the nearest orientation in 30 degrees increments. eg: 127 will be rounded to 120.
    - If the start and goal coordinates are valid, the script will display the path from the start to the goal using the A* algorithm.
    - If the start and goal coordinates are invalid, the script will prompt the user to enter the coordinates again.
    - The script will also display the time taken to find the path and the cost of the path.
    - The script will also display the path in the map.
    - For some start and goal coordinates, the script may not find a path. In such cases, the script will display a message saying "No path found".
    - The white cells represent the free space, the black cells represent the obstacles, the gray cells represent the clearance + the bloated region for the robot radius, the red cell represents the start node, the green cell represents the goal node, the blue cells represent the nodes visited, and the red cells represent the path.
    - The script will also save the path and the node creation as a video named A_star.mp4 in the current directory.
    - In the video, the start node is displayed in red, the goal node is displayed in green, the nodes visited are displayed in blue, and the path is displayed in red.
    - The script should not take more than 5 minutes to find the path.

## Contributors
Nitish Ravisankar Raveendran - rrnitish - 120385506\
Pranav ANV - anvpran - 1204886110

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

## Acknowledgements

Special thanks to the ENPM 661 course instructors for providing the project requirements and guidance.
