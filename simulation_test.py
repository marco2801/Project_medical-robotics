import time
import pybullet as p
import pybullet_data
import numpy as np

# Connect to PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Create the base plane for the simulation
plane_id = p.loadURDF("plane.urdf")

# Function to create a cylinder to represent the vessels
def create_vessel(radius, length, position, orientation):
    # Define the collision and visual shape
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=radius, length=length,
                                          rgbaColor=[0.8, 0.3, 0.3, 1.0], specularColor=[0.5, 0.5, 0.5])
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=radius, height=length)
    # Create the cylinder as a rigid body
    vessel_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return vessel_id

# Function to generate points along the edges of the vessels
def get_stiches(center, radius, num_stiches, offset=0.005):
    points = []
    for i in range(num_stiches):
        angle = 2 * np.pi * i / num_stiches
        x_position = center[0] + radius * np.cos(angle) + offset
        y_position = center[1] + radius * np.sin(angle) + offset  # Add an offset to see the points
        z_position = center[2]
        points.append([x_position, y_position, z_position])
    points = np.array(points)
    colors = np.array([[0, 1, 0]] * num_stiches)
    p.addUserDebugPoints(points, pointColorsRGB=colors, pointSize=5)
    return points


# Vessel parameters
vessel_radius = 0.05      # Radius of the vessels (5 cm)
vessel_length = 0.3       # Length of the vessels (30 cm)
gap = 0.02                # Space of 2 mm between vessel ends
num_suture_points = 7


# Position and orientation of the vessels to represent the anastomosis
# Position the vessels along the X-axis with 2 mm of space between the ends
position_vessel1 = [-vessel_length / 2 - gap / 2, 0, vessel_radius]
position_vessel2 = [vessel_length / 2 + gap / 2, 0, vessel_radius]
orientation_vessel = p.getQuaternionFromEuler([0, np.pi / 2, 0])  # Horizontal orientation

# Create the two vessels
vessel1_id = create_vessel(vessel_radius, vessel_length, position_vessel1, orientation_vessel)
vessel2_id = create_vessel(vessel_radius, vessel_length, position_vessel2, orientation_vessel)
center_vessel1 = [position_vessel1[0] + vessel_length / 2, position_vessel1[1], position_vessel1[2]]
center_vessel2 = [position_vessel2[0] - vessel_length / 2, position_vessel2[1], position_vessel2[2]]

# Generate the suture points
suture_vessel_1 = get_stiches(center_vessel1, vessel_radius, num_suture_points)
suture_vessel_2 = get_stiches(center_vessel2, vessel_radius, num_suture_points)

# Configure the camera for a better view
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=45, cameraPitch=-30,
                             cameraTargetPosition=[0, 0, vessel_radius])

# Simulation
while 1:
    p.stepSimulation()
    time.sleep(0.1)
