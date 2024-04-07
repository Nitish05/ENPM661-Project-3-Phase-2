#!/usr/bin/env python3
# Testing saving wheel velocities along with the path

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


RPM1 = 10
RPM2 = 20
WhR = 3.3
K = (2*np.pi*WhR)/60
# Ul = RPM1 * K
# Ur = RPM2 * K
# print(Ul, Ur)
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

# def obstacles(node):
#     x, y = node
#     Circ_center = (420, 120)
#     R = 600
#     Xc, Yc = Circ_center
#     y = abs(y - canvas_height)
#     obstacles = [
#         (x >= 1500 and x <= 1750 and y <= 2000 and y >= 1000), 
#         (x >= 2500 and x <= 2750 and y <= 1000 and y >= 0),
#         (((x - Xc)**2 + (y - Yc)**2) <= R**2),        
#     ]
#     return any(obstacles)

# def clearance(x, y, clearance):
#     clearance = clearance + robo_radius
#     Circ_center = (4200, 1200)
#     R = 600 + clearance
#     Xc, Yc = Circ_center
#     y = abs(y - canvas_height)
    
#     clearance_zones = [
#         (x >= 1500 - clearance and x <= 1750 + clearance and y <= 2000 + clearance  and y >= 1000 - clearance),
#         (x >= 2500 - clearance and x <= 2750 + clearance and y <= 1000 + clearance and y >= 0 - clearance),
#         (((x - Xc)**2 + (y - Yc)**2) <= R**2),
#         (x <= clearance or x >= canvas_width - clearance or y <= clearance or y >= canvas_height - clearance),
#     ]
#     return any(clearance_zones)


# def is_free(x, y):
#     x = int(round(x))
#     y = int(round(y))
#     if x >= 0 and x < canvas_width and y >= 0 and y < canvas_height:
#         return all(canvas[y, x] == free_space_color) or all(canvas[y, x] == (0, 255, 0)) or all(canvas[y, x] == (0, 0, 255))
#         # return all(canvas[y, x] == free_space_color) or all(canvas[y, x] == (0, 255, 0)) or all(canvas[y, x] == (0, 0, 255))
#     else:
#         return False

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

        # x += 0.5 * R * (action[0] + action[1]) * np.cos(thetan) * dt
        # y += 0.5 * R * (action[0] + action[1]) * np.sin(thetan) * dt
        # thetan += (R / L) * (action[1] - action[0]) * dt

        t = 0  # Reset t for each action

        while t < 1:
            t += dt
            # X_new += 0.5 * WhR * (action[0] + action[1]) * np.cos(thetan) * dt
            # Y_new += 0.5 * WhR * (action[0] + action[1]) * np.sin(thetan) * dt
            # thetan += (WhR / L) * (action[1] - action[0]) * dt
            Xs = X_new
            Ys = Y_new
            Vn = 0.5 * (Vl + Vr) 
            AVn = (Vr - Vl) / L
            vel.append((Vn, AVn))
            X_new += Vn * np.cos(thetan) * dt
            Y_new += Vn * np.sin(thetan) * dt
            thetan += AVn* dt
            print("X_new: ", X_new, "Y_new: ", Y_new, "thetan: ", thetan)
            # plt.plot([Xs, X_new], [Ys, Y_new], color = 'b', linewidth = 0.75)
            
            

       
        thetan_deg = np.rad2deg(thetan) % 360
        

      
        if is_free(X_new, Y_new):
            cost = ((initial_x - X_new)**2 + (initial_y - Y_new)**2)**0.5
            # cost = 1
            # neighbours.append(((X_new, Y_new, thetan_deg), cost, (action[0], action[1])))
            neighbours.append(((X_new,Y_new, thetan_deg), cost, vel))
            # cv2.circle(canvas, (int(round(X_new)), int(round(Y_new))), 1, (255, 0, 0), -1)
        
    return neighbours

# def get_neighbors(node):
#     dt = 0.1
#     neighbours = []
#     initial_x, initial_y, initial_theta = node
#     # Convert initial orientation to radians for calculation
#     initial_theta_rad = np.deg2rad(initial_theta)
    
#     action_set = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]

#     for action in action_set:
#         X_new, Y_new, thetan = initial_x, initial_y, initial_theta_rad

#         vel = []
#         Vl = action[0] * K
#         Vr = action[1] * K

#         t = 0  # Reset t for each action

#         while t < 1:
#             t += dt
#             Vn = 0.5 * (Vl + Vr)
#             AVn = (Vr - Vl) / L
#             vel.append((Vn, AVn))
#             X_new += Vn * np.cos(thetan) * dt
#             # Invert Y_new calculation to flip the y-axis
#             Y_new -= Vn * np.sin(thetan) * dt  # Subtract instead of adding to flip the axis
#             thetan += AVn * dt
            
#         # Convert thetan back to degrees for display or further calculations
#         thetan_deg = np.rad2deg(thetan) % 360

#         # No need to flip Y_new here, as we're using canvas coordinates directly
#         if is_free(X_new, canvas_height - Y_new):  # Flip y-axis for the check
#             cost = ((initial_x - X_new)**2 + (initial_y - Y_new)**2)**0.5
#             neighbours.append(((X_new, canvas_height - Y_new, thetan_deg), cost, vel))  # Store flipped y-axis value for consistency
        
#     return neighbours


# Function to check if the goal is reached
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
                # cv2.arrowedLine(canvas, (current_node[0][0], current_node[0][1]), (next_node[0], next_node[1]), (255, 0, 0), 1)
                # cv2.circle(canvas, (int(round(next_node[0])), int(round(next_node[1]))), 1, (0, 0, 255), -1)
                # canvas[int(round(next_node[1])), int(round(next_node[0]))] = (255, 0, 0)
                canvas_array[int(round(next_node[0])), int(round(next_node[1]))] = np.inf
                # canvas_array[next_node[0], next_node[1]] = np.inf
                came_from[next_node] = (current_node[0], action)
                count += 1
                if count%100 == 0:  
                    # cv2.resize(canvas, (600, 200))                  
                    out.write(canvas)
    return None, None, None  # Return None if no path is found

# def reconstruct_path(came_from, start, goal):
#     # Start with the goal node and work backwards to the start
#     current = goal
#     path = [current]
#     while current != start:
#         current = came_from[current]  # Move to the previous node in the path
#         path.append(current)
#     path.reverse()  # Reverse the path to go from start to goal
#     return path

# def reconstruct_path(came_from, start, goal):
#     current = goal
#     path_with_velocities = [(current, (0, 0))]
#     while current != start:
#         # Retrieve the current node and the velocities used to reach it
#         node_info = came_from[current]
#         prev_node, velocities = node_info[0], node_info[1]
#         path_with_velocities.append((current, velocities))
#         current = prev_node
#     path_with_velocities.reverse()  # Reverse to get the correct order from start to goal
#     return path_with_velocities

def reconstruct_path(came_from, start, goal):
    current = goal
    # Initialize path_with_velocities with the goal node; velocities can be set to (0,0) or to the actual values if available.
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



# Function to visualize the path
# def visualize_path(path):
#     V = []
#     for i in range(len(path)-1):
#         x, y, t = path[i][0]
#         xn, yn, tn = path[i+1][0]
#         vr = path[i][1][1]*K
#         vl = path[i][1][0]*K
#         # Linear_velocity = ((path[i][1][0] + path[i][1][1]) * R/2)/100
#         # Angular_velocity = float((((path[i][1][0] - path[i][1][1]) * R)/ L))
#         Angular_velocity = float(-(((vr - vl))/ L))
#         # Linear_velocity = float((((xn-x)**2 + (yn-y)**2)**0.5)/100)
#         Linear_velocity = ((vl + vr)*0.5)/100
#         # Angular_velocity = float(tn -t)


#         V.append((Linear_velocity, Angular_velocity))
#         print("Linear Velocity: ", Linear_velocity, "Angular Velocity: ", Angular_velocity)
        
#         cv2.arrowedLine(canvas, (int(round(x)), int(round(y))), (int(round(xn)),int(round(yn))), (0, 0, 255), 1)
#         out.write(canvas)
#     cv2.destroyAllWindows()       
#     for i in range(30):
#         out.write(canvas)
#     cv2.imshow('Path', canvas)
#     return V

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

# User input for step size, clearance distance and robot radius
# while True:
#     print("Step size should be between 1 and 10")
#     step_size = input("Enter the step size: ")                     # User input for step size
#     step_size = int(step_size)
#     if step_size > 0 and step_size <= 10:
#         break      

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

# Generate the map
for x in range(canvas_width):
    for y in range(canvas_height):
        if clearance(x, y, clearance_distance):
            canvas[y, x] = clearance_color
        if obstacles((x, y)):
            canvas[y, x] = obstacle_color

# Create a 3D array to store the visited nodes
# canvas_array = np.zeros((canvas_width, canvas_height, 1))
# for x in range(canvas_width):
#     for y in range(canvas_height):
#         if all(canvas[y, x] != free_space_color):
#             canvas_array[x, y, 0] = np.inf

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

canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

plt.figure(figsize=(10, 6))  # Size is arbitrary
plt.imshow(canvas_rgb)
plt.title('Canvas with OpenCV Drawings')
plt.axis('off')  # Hide the axes
plt.show()

for j in range(30):
    out.write(canvas)

# plt.imshow(canvas) 
# plt.show()

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