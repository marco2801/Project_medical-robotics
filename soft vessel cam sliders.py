from time import sleep # aggiunta
import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD) # aggiunta 
p.setGravity(0, 0, -9.81)  # Set gravity to a realistic value

plane_id = p.loadURDF("plane.urdf")

# vessel parameters
vessel_length = 0.02        # [m]
vessel_outer_radius = 0.005 #  (5 mm)

# Slider for number of suture points
suture_slider = p.addUserDebugParameter("Num Suture Points", 3, 20, 7)

# creation of the "needle" to test the interaction
def create_bullet(radius, length, position, orientation):
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=radius, length=length,
                                          rgbaColor=[1, 1, 0, 1.0], specularColor=[0.5, 0.5, 0.5])
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=radius, height=length)
    bullet_id = p.createMultiBody(baseMass=0.01, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return bullet_id

# Soft vessel creation
def create_soft_vessel(file_path,vtk_name, scale, position, orientation):
    soft_vessel_id = p.loadSoftBody(file_path,simFileName=vtk_name, basePosition=position, baseOrientation=orientation,
                                    scale=scale, mass=1, springElasticStiffness=100000, useSelfCollision=False,
                                    collisionMargin=0.01)
    
    return soft_vessel_id

#lambda = Enu/(1+nu)/(1-2nu)
#mu = E /2/(1+nu)

# built of stitches we have to put them in the right position (next version)
def get_stitches(center, radius, num_stitches, offset=0.0005):
    debug_items = []
    for i in range(num_stitches):
        angle = 2 * np.pi * i / num_stitches
        x_position = center[0]
        y_position = center[1] + (radius + offset) * np.sin(angle)
        z_position = center[2] + (radius + offset) * np.cos(angle)
        point_id = p.addUserDebugPoints([[x_position, y_position, z_position]],
                                        pointColorsRGB=[[0, 1, 0]], pointSize=5)
        debug_items.append(point_id)
    return debug_items

# load the soft vessel
file_path = "artery1000.obj"  # Make sure to have this file
vtk_name = "artery1000.vtk"  # Make sure to have this file

# positioning of the vessel
scale_factor = 0.001
position_vessel = [0, 0,  2*vessel_outer_radius ]  # positioned on the floor
orientation_vessel = p.getQuaternionFromEuler([0, np.pi/2, 0])

# creation of one vessel
vessel1 = create_soft_vessel(file_path, vtk_name,scale_factor, position_vessel, orientation_vessel)

# Slider for controlling the z position of the bullet
bullet_slider = p.addUserDebugParameter("Bullet Position Z", 0.5* vessel_outer_radius, 0.1, 3 * vessel_outer_radius + 0.1)

# Initially create the bullet
bullet_position = [vessel_length/2, vessel_outer_radius/2, 3* vessel_outer_radius]
bullet_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Vertical orientation
bullet_id = create_bullet(0.001, vessel_length, bullet_position, bullet_orientation)

# Variables to track the suture points IDs
debug_points_ids = []  # List for the suture points IDs
previous_suture_points = 0  # Check if the number has changed

# Set up the camera for a better view
camera_yaw = p.addUserDebugParameter("Camera Yaw", 0, 360, 90)
camera_pitch = p.addUserDebugParameter("Camera Pitch", -180, 180, -5)

# Simulation loop
while True:
    p.stepSimulation()

    # Read values from the sliders
    num_suture_points = int(round(p.readUserDebugParameter(suture_slider)))
    new_bullet_z = p.readUserDebugParameter(bullet_slider)
    yaw = p.readUserDebugParameter(camera_yaw)
    pitch = p.readUserDebugParameter(camera_pitch)
    p.resetDebugVisualizerCamera(cameraDistance=0.1, cameraYaw=yaw, cameraPitch=pitch,
                                 cameraTargetPosition=[0, 0,  vessel_outer_radius ])

    # Update the bullet's position if the slider has changed
    if new_bullet_z != bullet_position[2]:
        # Remove the previous bullet
        p.removeBody(bullet_id)

        # Update position and recreate the bullet
        bullet_position = [vessel_length/2, vessel_outer_radius/2, new_bullet_z]
        bullet_id = create_bullet(0.001, vessel_length, bullet_position, bullet_orientation)

    # Generate new suture points if the number changes
    if num_suture_points != previous_suture_points:
        # Remove previous suture points
        for point_id in debug_points_ids:
            p.removeUserDebugItem(point_id)

        # Generate new suture points around the cylinder's edge
        center_vessel1 = [position_vessel[0], position_vessel[1], position_vessel[2]]
        debug_points_ids = get_stitches(center_vessel1, vessel_outer_radius, num_suture_points)

        previous_suture_points = num_suture_points  # Update the number of suture points

    # Check for collisions and highlight interaction points


    sleep(0.1)