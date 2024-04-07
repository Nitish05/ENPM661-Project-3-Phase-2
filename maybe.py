#!/usr/bin/env python3
# Testing curves

# Import necessary libraries



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pynput import keyboard
import cv2
import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
import time

WhR = 3.3
K = (2*np.pi*WhR)/60
L = 28.7

canvas_height = 200
canvas_width = 600
robo_radius = 22.0

clearance_color = (127, 127, 127)
obstacle_color = (0, 0, 0)
free_space_color = (255, 255, 255)
threshold = 1.0

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(2)  # Give some time for the publisher to set up

    def publish_velocity(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

# Initialize a white canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

def obstacles(node):
    x, y = node
    Circ_center = (420, 120)
    R = 60
    Xc, Yc = Circ_center
    y = abs(y - canvas_height)
    obstacles = [
        (x >= 150 and x <= 175 and y <= 200 and y >= 100), 
        (x >= 250 and x <= 275 and y <= 100 and y >= 0),
        (((x - Xc)**2 + (y - Yc)**2) <= R**2),        
    ]
    return any(obstacles)

def clearance(x, y, clearance):
    clearance = clearance + robo_radius
    Circ_center = (420, 120)
    R = 60 + clearance
    Xc, Yc = Circ_center
    y = abs(y - canvas_height)
    
    clearance_zones = [
        (x >= 150 - clearance and x <= 175 + clearance and y <= 200 + clearance  and y >= 100 - clearance),
        (x >= 250 - clearance and x <= 275 + clearance and y <= 100 + clearance and y >= 0 - clearance),
        (((x - Xc)**2 + (y - Yc)**2) <= R**2),
        (x <= clearance or x >= canvas_width - clearance or y <= clearance or y >= canvas_height - clearance),
    ]
    return any(clearance_zones)



def is_free(x, y):
    x = int(round(x))
    y = int(round(y))
    if x >= 0 and x < canvas_width and y >= 0 and y < canvas_height:
        if canvas_array[x, y] == 0:
            return True
    else:
        return False

    
def get_neighbors(node):
    dt = 0.1
    neighbours = []
    initial_x, initial_y, initial_theta = node
    # Convert initial orientation to radians for calculation
    initial_theta_rad = np.deg2rad(initial_theta)
    
    action_set = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]
    # action_set = [(0, Ul), (Ul, 0), (Ul, Ul), (0, Ur), (Ur, 0), (Ur, Ur), (Ul, Ur), (Ur, Ul) ]

    for action in action_set:
        X_new, Y_new, thetan = initial_x, initial_y, initial_theta_rad

        vel = []
        Vl = action[0] * K
        Vr = action[1] * K

   

        t = 0  # Reset t for each action

        while t < 1:
            t += dt
  
            Xs = X_new
            Ys = Y_new
            Vn = 0.5 * (Vl + Vr)
            AVn = (Vr - Vl) / L
            vel.append((Vn, AVn))            
            X_new += Vn * np.cos(thetan) * dt
            Y_new += Vn * np.sin(thetan) * dt
            thetan += AVn* dt
       
            if is_free(X_new, Y_new):
                # pass
                cv2.line(canvas, (int(round(Xs)), int(round(Ys))), (int(round(X_new)), int(round(Y_new))), (255, 0, 0), 1)
            
        
            

       
        thetan_deg = np.rad2deg(thetan) % 360
        

      
        if is_free(X_new, Y_new):
            cost = ((initial_x - X_new)**2 + (initial_y - Y_new)**2)**0.5
            neighbours.append(((X_new,Y_new, thetan_deg), cost, vel))
           
        
    return neighbours

def check_goal_reached(current_node, goal):
    distance = ((current_node[0] - goal[0]) ** 2 + (current_node[1] - goal[1]) ** 2) ** 0.5
    return distance < threshold

# A* algorithm
def a_star(start, goal):
    pq = PriorityQueue()
    cost_to_goal = ((goal[0] - start[0])**2 + (goal[1] - start[1])**2)**0.5 # Heuristic cost
    pq.put((cost_to_goal, (start, 0)))
    came_from = {start: None}
    cost_so_far = {start: cost_to_goal}
    count = 0

    while not pq.empty():
        current_cost, current_node = pq.get()
        if check_goal_reached(current_node[0], goal):  # Check if the goal is reached
            print("Goal Reached")
            print("Cost to Goal: " , cost_so_far[current_node[0]])
            goal = current_node[0]
            return came_from, cost_so_far, goal  # Return the path
            
        for next_node, cost, action in get_neighbors(current_node[0]):  # Get the neighbors of the current node
            cost_to_go = ((goal[0] - next_node[0])**2 + (goal[1] - next_node[1])**2)**0.5
            theta_normalized = next_node[2] % 360
            theta_index = theta_normalized // 30
            new_cost = current_node[1] + cost + cost_to_go
            nc = current_node[1] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:   # Check if the new cost is less than the cost so far
                cost_so_far[next_node] = nc  # Update the cost so far
                priority = new_cost   # Calculate the priority
                pq.put((priority, (next_node, nc)))   # Add the node to the priority queue
                canvas[int(round(next_node[1])), int(round(next_node[0]))] = (255, 0, 0)
                canvas_array[int(round(next_node[0])), int(round(next_node[1]))] = np.inf
                came_from[next_node] = (current_node[0], action)
                count += 1
                if count%100 == 0:  
                    # cv2.resize(canvas, (600, 200))                  
                    out.write(canvas)
    return None, None, None  # Return None if no path is found



def reconstruct_path(came_from, start, goal):
    current = goal
 
    path_with_velocities = [(current, came_from[current][1] if current in came_from else (0, 0))]
    
    # Loop until the start node is reached.
    while current != start:
        # Retrieve the current node and the velocities used to reach it.
        node_info = came_from[current]
        prev_node, velocities = node_info  # Assuming node_info is a tuple of (prev_node, velocities)
        
        # Move to the previous node in the path.
        current = prev_node
        
        # Add the previous node and its velocities to the path list.
        path_with_velocities.append((current, velocities))
    
    # Reverse the path to get the correct order from start to goal.
    path_with_velocities.reverse()
    
    return path_with_velocities





def visualize_path(path):
    V = []
    for i in range(len(path) - 1):
        current_node, velocities = path[i]
        next_node = path[i + 1][0]
        x, y, t = current_node
        xn, yn, tn = next_node
        
        # Assuming `velocities` is a list of (linear_velocity, angular_velocity) tuples
        for linear_vel, angular_vel in velocities:
            # Process each velocity pair as needed
            print("Linear Velocity: ", linear_vel/100, "Angular Velocity: ", -angular_vel)
            V.append((linear_vel/100, -angular_vel))
            
            # For visualization purposes, you might only want to draw the move from the first or last velocity pair
            cv2.arrowedLine(canvas, (int(round(x)), int(round(y))), (int(round(xn)),int(round(yn))), (0, 0, 255), 1)
            out.write(canvas)
            # Adjust your drawing logic here accordingly

    # Finalize your visualization here if necessary
    cv2.destroyAllWindows()       
    for i in range(30):
        out.write(canvas)
    cv2.imshow('Path', canvas)
    return V

print('''
_____________________________________
    __          __                   
    / |       /    )                 
---/__|-------\------_/_----__---)__-
  /   |        \     /    /   ) /   )
_/____|____(____/___(_ __(___(_/_____
                                     
''')
     

while True:
    print("Clearance distance should be a positive number")
    clearance_distance = input("Enter the clearance distance: ")           # User input for clearance distance
    if clearance_distance.isdigit() and int(clearance_distance) >= 0:
        clearance_distance = int(clearance_distance)
        break
    
# while True:
#     print("Robot radius should be a positive number")
#     robo_radius = input("Enter the robot radius: ")                       # User input for robot radius
#     if robo_radius.isdigit() and int(robo_radius) >= 0:
#         robo_radius = int(robo_radius)
#         break

print("\nGenerating the map...\n")

while True:
    print("Enter the RPM values for the left and right wheels eg. 10, 20")
    RPM1 = input("Enter the RPM 1: ")
    RPM2 = input("Enter the RPM 2: ")
    if RPM1.isdigit() and RPM2.isdigit():
        RPM1 = int(RPM1)
        RPM2 = int(RPM2)
        break
# Generate the map
for x in range(canvas_width):
    for y in range(canvas_height):
        if clearance(x, y, clearance_distance):
            canvas[y, x] = clearance_color
        if obstacles((x, y)):
            canvas[y, x] = obstacle_color

canvas_array = np.zeros((canvas_width, canvas_height))
for x in range(canvas_width):
    for y in range(canvas_height):
        if all(canvas[y, x] != free_space_color):
            canvas_array[x, y] = np.inf
        else:
            canvas_array[x, y] = 0


out = cv2.VideoWriter('A_star.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

C = clearance_distance + robo_radius + 1
Xc = canvas_width - C
Yc = canvas_height - C
plt.imshow(canvas)
# User input for start and goal nodes        
while True:
    print(f"The start node and goal node should be within the canvas dimensions ({C}-{Xc}, {C}-{Yc}) and not inside an obstacle.\n")
    Xi = input("Enter the start node X: ")
    Yi = input("Enter the start node Y: ")
    Ti = input("Enter the start node Angle: ")
    Xi = int(Xi)
    Yi = int(Yi)
    Ti = int(Ti)
    
    if not (Xi < 0 or Xi >= canvas_width or Yi < 0 or Yi >= canvas_height):    # Check if the start node is within the canvas dimensions
        if is_free(Xi, Yi):
            break
        else:
            print("Start node is inside an obstacle")
    else:
        print("Start node is out of bounds.")



while True:
    Xg = input("Enter the goal node X: ")
    Yg = input("Enter the goal node Y: ")
    To = input("Enter the goal node Angle: ")
    Xg = int(Xg)
    Yg = int(Yg)
    To = int(To)

    if not (Xg < 0 or Xg >= canvas_width or Yg < 0 or Yg >= canvas_height):    # Check if the goal node is within the canvas dimensions
        if is_free(Xg, Yg): 
            break
        else:
            print("Goal node is inside an obstacle")
    else:
        print("Goal node is inside an obstacle or out of bounds.")

# Round the angles to the nearest multiple of 30
Ti = Ti % 360
Ti = round(Ti/30)*30                                          # Round the angle to the nearest multiple of 30
To = To % 360
To = round(To/30)*30                                         # Round the angle to the nearest multiple of 30

print("Start Node: ", (int(Xi), int(Yi), int(Ti)))
print("Goal Node: ", (int(Xg), int(Yg), int(To)))

Yi = abs(canvas_height - int(Yi))
start_node = (int(Xi), int(Yi), int(Ti))

Yg = abs(canvas_height - int(Yg))
goal_node = (int(Xg), int(Yg), int(To))

canvas_array = np.zeros((canvas_width, canvas_height))
for x in range(canvas_width):
    for y in range(canvas_height):
        if all(canvas[y, x] != free_space_color):
            canvas_array[x, y] = np.inf
        else:
            canvas_array[x, y] = 0

cv2.circle(canvas, (Xi, Yi), 2, (0, 0, 255), -1)
cv2.circle(canvas, (Xg, Yg), 2, (0, 255, 0), -1)

for j in range(30):
    out.write(canvas)


start_time = time.time()
came_from, cost_so_far, goal = a_star(start_node, goal_node)
if came_from is None:
    print("No path found")
    end_time = time.time()
    execution_time = end_time - start_time         # Calculate the execution time
    print("Execution time: %.4f seconds" % execution_time)
    exit()
path = reconstruct_path(came_from, start_node, goal)
velocities = visualize_path(path)

end_time = time.time()
execution_time = end_time - start_time

for i in range(30):
    out.write(canvas)

out.release()
print("Execution time: %.4f seconds" % execution_time)
cv2.waitKey(0)

def main(velocities):
    rclpy.init()
    velocity_publisher = VelocityPublisher()
    try:
        for velocity in velocities:
            linear_vel, angular_vel = velocity
            velocity_publisher.publish_velocity(linear_vel, angular_vel)
            time.sleep(0.1) 
        velocity_publisher.publish_velocity(0.0, 0.0)  # Stop the robot
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(velocities)

cv2.destroyAllWindows()