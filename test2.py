import numpy as np
import cv2

RPM1 = 5
RPM2 = 10

R = 3.3
K = (2*np.pi*R)/60
L = 30.6
Ul = RPM1 * K
Ur = RPM2 * K
canvas_height = 200
canvas_width = 600

canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

# def get_neighbors(node):
#     x, y , theta = node
#     # neighbours = []
#     dt = 1
#     # action_set = [theta, theta +30, theta -30, theta +60, theta -60]  # Action set for the robot
#     action_set = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]

#     for action in action_set:
#         Ul = action[0] * K
#         Ur = action[1] * K

#         delta_theta = (R/L) * (Ur - Ul) * dt

#         if delta_theta == 0:  # Moving straight
#             delta_x = R * Ul * np.cos(np.deg2rad(theta)) * dt
#             delta_y = R * Ul * np.sin(np.deg2rad(theta)) * dt
#         else:
#             R_curvature = L/2 * ((Ul + Ur) / (Ur - Ul))
#             delta_x = R_curvature * (np.sin(np.deg2rad(theta + np.rad2deg(delta_theta))) - np.sin(np.deg2rad(theta)))
#             delta_y = R_curvature * (-np.cos(np.deg2rad(theta + np.rad2deg(delta_theta))) + np.cos(np.deg2rad(theta)))

#         x_new = x + delta_x
#         y_new = y + delta_y
#         x_new = int(round(x_new))
#         y_new = int(round(y_new))
#         theta_new = (theta + np.rad2deg(delta_theta)) % 360
#         print(x_new, y_new, theta_new)
#         cv2.circle(canvas, (x_new, y_new), 1, (0, 0, 255), -1)

def get_neighbors(node):
    dt = 0.1
    initial_x, initial_y, initial_theta = node
    # Convert initial orientation to radians for calculation
    initial_theta_rad = np.deg2rad(initial_theta)
    
    action_set = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]
    # action_set = [(0, Ul), (Ul, 0), (Ul, Ul), (0, Ur), (Ur, 0), (Ur, Ur), (Ul, Ur), (Ur, Ul) ]

    for action in action_set:
        x, y, thetan = initial_x, initial_y, initial_theta_rad
        neighbours = []

        # x += 0.5 * R * (action[0] + action[1]) * np.cos(thetan) * dt
        # y += 0.5 * R * (action[0] + action[1]) * np.sin(thetan) * dt
        # thetan += (R / L) * (action[1] - action[0]) * dt
        t = 0  # Reset t for each action

        while t < 1:
            t += dt
            x += 0.5 * R * (action[0] + action[1]) * np.cos(thetan) * dt
            y += 0.5 * R * (action[0] + action[1]) * np.sin(thetan) * dt
            thetan += (R / L) * (action[1] - action[0]) * dt

        # Convert thetan back to degrees for display or further calculations
        thetan_deg = np.rad2deg(thetan) % 360
        print(x, y, thetan_deg)
        neighbours.append((x, y, thetan_deg))
        

        # Optional: Draw a circle for visualization
        cv2.circle(canvas, (int(round(x)), int(round(y))), 1, (0, 0, 255), -1)
    return neighbours

 
        

x = 25
y = 25
theta = 0
cv2.circle(canvas, (x, y), 1, (0, 0, 0), -1)



get_neighbors((x, y, theta))

new_height = canvas_height * 2
new_width = canvas_width * 2

resized_canvas = cv2.resize(canvas, (new_width, new_height), interpolation=cv2.INTER_AREA)

cv2.imshow("Canvas", canvas)
cv2.waitKey(0)
cv2.imshow("Resized Canvas", resized_canvas)
cv2.waitKey(0)
