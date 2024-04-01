import cv2
import numpy as np
import math

# Initialize an empty canvas
canvas_height = 1000
canvas_width = 1000
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

def plot_curve_opencv(Xi, Yi, Thetai, UL, UR, canvas):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = np.deg2rad(Thetai)
    scale = 1000  # Scale factor for better visualization

    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        # Drawing the line on canvas
        cv2.line(canvas, (int(Xs*scale), int(Ys*scale)), (int(Xn*scale), int(Yn*scale)), (255, 0, 0), 2)
    Thetan = np.rad2deg(Thetan)
    return Xn, Yn, Thetan

# Define actions
actions = [[5,5], [10,10], [5,0], [0,5], [5,10], [10,5]]

# Initial configuration for the curve
X1 = plot_curve_opencv(0, 0, 45, 5, 5, canvas)  # Example starting configuration

# Generate and draw subsequent curves based on the actions
for action in actions:
    X1 = plot_curve_opencv(X1[0], X1[1], X1[2], action[0], action[1], canvas)

# Display the result
cv2.imshow('Curves in OpenCV', canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
