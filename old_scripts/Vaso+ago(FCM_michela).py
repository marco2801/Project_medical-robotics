# Vaso+ago
import pybullet as p
from time import sleep
import numpy as np
import pybullet_data

def create_needle_from_stl(file_path, position, orientation, scale=0.1):
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    needle_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return needle_id

# Connessione a PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
# Configura la gravità e aggiungi il piano
p.setGravity(0, 0, -9.81)
plane_id = p.loadURDF("plane.urdf")

#%% Vaso
vessel_outer_radius = 0.5
thickness=0.01
vessel1 = p.loadSoftBody("Arteria_piena.obj", simFileName="Arteria_piena.vtk", basePosition=[0,-vessel_outer_radius, 2*vessel_outer_radius], baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]),
                        scale=0.1, mass=4, useNeoHookean=1, NeoHookeanMu=5000, NeoHookeanLambda=1000, NeoHookeanDamping=0.01, useSelfCollision=1, frictionCoeff=0.5, collisionMargin=0.01)
#vessel2=p.loadSoftBody("Arteria_piena.obj", simFileName="Arteria_piena.vtk", basePosition=[2, 0, 2*0.5], baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]),
                      # scale=0.1, mass=4, useNeoHookean=1, NeoHookeanMu=5000, NeoHookeanLambda=1000, NeoHookeanDamping=0.01, useSelfCollision=1, frictionCoeff=0.5, collisionMargin=0.01)

# Caricamento ago come corpo rigido

# Parametri del semicerchio
radius = 0.33  # Raggio del semicerchio in metri (33 mm)
angle = 0  # Angolo iniziale

# Parametri dell'ago
#needle_position = [0, vessel_outer_radius/2, (2*vessel_outer_radius)-thickness]# Posizione del centro fisso
needle_position =  [ -(0.3/2+0.03),0, (2*vessel_outer_radius)+0.01]  # Posizione iniziale
needle_orientation = p.getQuaternionFromEuler([np.pi, 0, np.pi/2])  # Orientamento iniziale
needle_id = create_needle_from_stl("ago_66mm.stl", needle_position, needle_orientation, scale=0.01)

# Sliders per controllare la velocità di rotazione
rotation_speed_slider = p.addUserDebugParameter("Rotation Speed",0, 10.0, 0.5)

# Centro del semicerchio
semicircle_center = [-1, 2, 2*vessel_outer_radius]   #posizione telcamere
needle_center=[ -(0.3/2+0.03),0, (2*vessel_outer_radius)+0.01]  # Posizione del centro fisso

# Aggiungi sliders per posizionare la telecamera esternamente
camera_yaw_slider = p.addUserDebugParameter("Camera Yaw", 0, 360, 145)
camera_pitch_slider = p.addUserDebugParameter("Camera Pitch", -180, 180, -5)
camera_distance_slider = p.addUserDebugParameter("Camera Distance", 0.1, 10, 1)

num_suture=12
current_suture=1
suture_angle=2*np.pi/num_suture
angle=0


while p.isConnected():
    # Simulazione
         p.stepSimulation()
    # Ottieni valori dagli sliders
         rotation_speed = p.readUserDebugParameter(rotation_speed_slider)
         yaw = p.readUserDebugParameter(camera_yaw_slider)
         pitch = p.readUserDebugParameter(camera_pitch_slider)
         distance = p.readUserDebugParameter(camera_distance_slider)
         angle += rotation_speed * (1 / 240)
         needle_center=[ -(0.3/2+0.03),0, (2*vessel_outer_radius)+0.01]
         needle_orientation=p.getQuaternionFromEuler([np.pi+angle, 0, np.pi/2])
         p.resetBasePositionAndOrientation(needle_id, needle_center, needle_orientation)
         print(angle)
         while angle>=(5/4)*np.pi and current_suture<=num_suture:
            current_angle=current_suture*suture_angle
            angle1=0
            print(current_angle)
            print(current_suture)
            while angle1<(5/4)*np.pi and current_suture<=num_suture:
             p.stepSimulation()
             rotation_speed = p.readUserDebugParameter(rotation_speed_slider)
             yaw = p.readUserDebugParameter(camera_yaw_slider)
             pitch = p.readUserDebugParameter(camera_pitch_slider)
             distance = p.readUserDebugParameter(camera_distance_slider)
             angle1+=rotation_speed * (1 / 240)
             needle_orientation=p.getQuaternionFromEuler([np.pi+angle1,current_angle,np.pi/2])
             needle_center=[-(0.3/2+0.03),((vessel_outer_radius)+0.01)*np.sin(current_angle),((vessel_outer_radius)+0.01)*np.cos(current_angle)+(vessel_outer_radius)]
             p.resetBasePositionAndOrientation(needle_id, needle_center, needle_orientation)
             p.resetDebugVisualizerCamera(
             cameraDistance=distance,
             cameraYaw=yaw,
             cameraPitch=pitch,
             cameraTargetPosition=semicircle_center
             )
             sleep(1./240.)
            current_suture+=1
             
         
        # Configura telecamera
         p.resetDebugVisualizerCamera(
         cameraDistance=distance,
         cameraYaw=yaw,
         cameraPitch=pitch,
         cameraTargetPosition=semicircle_center
         )
         
         sleep(1./240.)
