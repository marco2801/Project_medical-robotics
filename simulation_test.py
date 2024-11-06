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
def get_stiches(center, radius, num_stiches, offset=0.000):
    debug_items = []
    for i in range(num_stiches):
        angle = 2 * np.pi * i / num_stiches
        x_position = center[0]
        y_position = center[1] + radius * np.sin(angle) + offset  # Add an offset to see the points
        z_position = center[2]+ radius * np.cos(angle) + offset
        point_id = p.addUserDebugPoints([[x_position, y_position, z_position]],
                                        pointColorsRGB=[[0, 1, 0]], pointSize=5)
        debug_items.append(point_id)
    return debug_items


# Vessel parameters
vessel_radius = 0.05      # Radius of the vessels (5 cm)
vessel_length = 0.3       # Length of the vessels (30 cm)
gap = 0.02                # Space of 2 mm between vessel ends

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
#slider for the number of sutere points
suture_slider = p.addUserDebugParameter("Num Suture Points", 3, 20, 7)

# variables for traking ids
debug_points_ids = []  # point ids list
previous_suture_points = 0  # check if the number has changed

# Configure the camera for a better view
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=45, cameraPitch=-30,
                             cameraTargetPosition=[0, 0, vessel_radius])

# Simulation
while True:
    p.stepSimulation()
    # read the value in the slider
    num_suture_points = int(round(p.readUserDebugParameter(suture_slider)))

    # if the number of steatches change remove the old points
    if num_suture_points != previous_suture_points:
        # removal
        for point_id in debug_points_ids:
            p.removeUserDebugItem(point_id)

        # update the new points
        debug_points_ids = get_stiches(center_vessel1, vessel_radius, num_suture_points)
        debug_points_ids += get_stiches(center_vessel2, vessel_radius, num_suture_points)


        previous_suture_points = num_suture_points

    time.sleep(0.1)
