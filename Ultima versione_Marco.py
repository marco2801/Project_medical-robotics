##%% caricamenyo ado come solid body

import pybullet as p
import pybullet_data

from simulation_sliders import vessel_length

# Connessione a PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Configura la gravità e aggiungi il piano
p.setGravity(0, 0, -9.81)
plane_id = p.loadURDF("plane.urdf")

# Carica il tuo modello STL o OBJ come corpo rigido
collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                            fileName="ago_66mm.stl",
                                            meshScale=[0.01, 0.01, 0.01])  # Scala il modello se necessario

visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                      fileName="ago_66mm.stl",
                                      meshScale=[0.01, 0.01, 0.01])

# Crea il corpo rigido
rigid_body_id = p.createMultiBody(baseMass=1,  # Specifica la massa
                                  baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id,
                                  basePosition=[0, 0, 1])  # Posiziona il corpo nello spazio

# Simulazione
while p.isConnected():
    p.stepSimulation()

#%% doppio vaso soft
import pybullet as p
from time import sleep
import numpy as np
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
vessel_outer_radius = 0.5

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -9.81)

#planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,0])

#boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)
#base=loadURDF()
vessel1 = p.loadSoftBody("Arteria_piena.obj", simFileName="Arteria_piena.vtk", basePosition=[0, 0, 2*0.5], baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]),
                        scale=0.1, mass=4, useNeoHookean=1, NeoHookeanMu=5000, NeoHookeanLambda=1000, NeoHookeanDamping=0.01, useSelfCollision=1, frictionCoeff=0.5, collisionMargin=0.01)
#vessel2=p.loadSoftBody("Arteria_piena.obj", simFileName="Arteria_piena.vtk", basePosition=[2, 0, 2*0.5], baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]),
                      # scale=0.1, mass=4, useNeoHookean=1, NeoHookeanMu=5000, NeoHookeanLambda=1000, NeoHookeanDamping=0.01, useSelfCollision=1, frictionCoeff=0.5, collisionMargin=0.01)

p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
#balldata=p.getMeshData(artery700)

#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")

camera_yaw = p.addUserDebugParameter("Camera Yaw", 0, 360, 145)
camera_pitch = p.addUserDebugParameter("Camera Pitch", -180, 180, -5)
camera_distance = p.addUserDebugParameter("Camera Distance", 0.1, 10, 1)

# Check if the vessel is touching the ground


# Calculate the distance from the bottom of the AABB to the plane


while p.isConnected():

    p.stepSimulation()
    yaw = p.readUserDebugParameter(camera_yaw)
    pitch = p.readUserDebugParameter(camera_pitch)
    distance=p.readUserDebugParameter(camera_distance)
    p.resetDebugVisualizerCamera(cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch,
                                 cameraTargetPosition=[2, 2,  vessel_outer_radius])
    #there can be some artifacts in the visualizer window,
    #due to reading of deformable vertices in the renderer,
    #while the simulators updates the same vertices
    #it can be avoided using
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    #but then things go slower
    p.setGravity(0,0,-9.81)
    #sleep(1./240.)

#p.resetSimulation()
#p.stopStateLogging(logId)




#%% rotazione ago
import pybullet as p
import pybullet_data
import numpy as np
import time

# Connessione a PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Configurazione dell'ambiente
p.loadURDF("plane.urdf", [0, 0, -10])

# Funzione per caricare un oggetto da un file STL
def create_needle_from_stl(file_path, position, orientation, scale=0.1):
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    needle_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return needle_id

# Parametri del semicerchio
radius = 0.33  # Raggio del semicerchio in metri (33 mm)
angle = 0  # Angolo iniziale
angle_increment = 0.05  # Incremento angolare per la rotazione

# Parametri dell'ago
needle_position = [0, 0, 0]  # Posizione del centro fisso
needle_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Orientamento iniziale
needle_id = create_needle_from_stl("ago_66mm.stl", needle_position, needle_orientation, scale=0.1)

# Sliders per controllare la velocità di rotazione
rotation_speed_slider = p.addUserDebugParameter("Rotation Speed", -2.0, 2.0, 0.5)

# Aggiungi sliders per posizionare la telecamera esternamente


# Centro del semicerchio
semicircle_center = [0, 0, 0]  # Centro fisso del semicerchio

# Ciclo di simulazione
while p.isConnected():
    # Ottieni la velocità di rotazione dallo slider
    rotation_speed = p.readUserDebugParameter(rotation_speed_slider)

    # Aumenta l'angolo di rotazione
    angle += rotation_speed * (1 / 240)  # Incremento angolare


    # Calcola la posizione della punta dell'ago (semicerchio)
    x_position = semicircle_center[0] + radius * np.cos(angle)  # Posizione X sulla traiettoria circolare
    y_position = semicircle_center[1] + radius * np.sin(angle)  # Posizione Y sulla traiettoria circolare
    z_position = needle_position[2]  # Mantieni la stessa altezza del centro

    # Calcola il nuovo orientamento (la punta dell'ago deve essere rivolta verso la direzione del movimento)
    needle_orientation = p.getQuaternionFromEuler([angle, 0, 0])  # Ruota lungo Z

    # Aggiorna solo l'orientamento dell'ago, mantenendo il centro fisso
    p.resetBasePositionAndOrientation(needle_id, [x_position, y_position, z_position], needle_orientation)

    # Ottieni la posizione della telecamera dagli sliders


    # Configura la telecamera come un osservatore esterno



    # Simulazione
    p.stepSimulation()
    time.sleep(1 / 240)






#%% verifica di traiettoria circolare
import pybullet as p
import pybullet_data
import numpy as np
import time

# Connessione a PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Configurazione dell'ambiente
p.loadURDF("plane.urdf", [0, 0, -10])

# Funzione per caricare un oggetto da un file STL
def create_needle_from_stl(file_path, position, orientation, scale=0.1):
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    needle_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return needle_id

# Parametri del semicerchio
radius = 0.33  # Raggio del semicerchio in metri (33 mm)
angle = 0  # Angolo iniziale
angle_increment = 0.05  # Incremento angolare per la rotazione

# Parametri dell'ago
needle_position = [0, 0, 0]  # Posizione del centro fisso
needle_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Orientamento iniziale
needle_id = create_needle_from_stl("ago_66mm.stl", needle_position, needle_orientation, scale=0.1)

# Sliders per controllare la velocità di rotazione
rotation_speed_slider = p.addUserDebugParameter("Rotation Speed", -5.0, 5.0, 0.5)

# Variabili per tracciare la traiettoria
trajectory_points = []  # Lista per salvare i punti della traiettoria
trajectory_point_ids = []  # Lista per gli ID dei punti nella simulazione

# Centro del semicerchio
semicircle_center = [0, 0, 0]  # Centro fisso del semicerchio

# Ciclo di simulazione
while p.isConnected():
    # Ottieni la velocità di rotazione dallo slider
    rotation_speed = p.readUserDebugParameter(rotation_speed_slider)

    # Aumenta l'angolo di rotazione
    angle += rotation_speed * (1 / 240)  # Incremento angolare

    # Calcola la posizione della punta dell'ago (semicerchio)
    x_position = semicircle_center[0] + radius * np.cos(angle)  # Posizione X sulla traiettoria circolare
    y_position = semicircle_center[1] + radius * np.sin(angle)  # Posizione Y sulla traiettoria circolare
    z_position = needle_position[2]  # Mantieni la stessa altezza del centro

    # Calcola il nuovo orientamento (la punta dell'ago deve essere rivolta verso la direzione del movimento)
    needle_orientation = p.getQuaternionFromEuler([angle, 0, 0])  # Ruota lungo Z

    # Aggiorna solo l'orientamento dell'ago, mantenendo il centro fisso
    p.resetBasePositionAndOrientation(needle_id, [x_position, y_position, z_position], needle_orientation)

    # Aggiungi il punto della traiettoria
    trajectory_points.append([x_position, y_position, z_position])  # Aggiungi il punto alla lista

    # Aggiorna la visualizzazione della traiettoria
    if len(trajectory_points) > 1:  # Non disegnare punti se non ci sono abbastanza punti
        last_point = trajectory_points[-1]
        prev_point = trajectory_points[-2]
        point_id = p.addUserDebugLine(prev_point, last_point, lineColorRGB=[0, 1, 0], lineWidth=7)
        trajectory_point_ids.append(point_id)

    # Ottieni la posizione della telecamera dagli sliders

    # Configura la telecamera come un osservatore esterno


    # Simulazione
    p.stepSimulation()
    time.sleep(1 / 240)



# %% buca o non buca
import pybullet as p
import pybullet_data
import numpy as np
import time

# Connessione a PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, -9.81)

# Caricamento del piano
p.loadURDF("plane.urdf", [0, 0, 0])

# Funzione per caricare un oggetto da un file STL
def create_needle_from_stl(file_path, position, orientation, scale=0.01):
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=file_path, meshScale=[scale, scale, scale])
    needle_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return needle_id

# Parametri del vaso
vessel_outer_radius = 0.5  # Raggio esterno del vaso (50 mm)
vessel_length = 1.0  # Lunghezza del vaso


# Parametri del semicerchio
radius = 0.33  # Raggio del semicerchio in metri (33 mm)
angle = 0  # Angolo iniziale
needle_center=[0.5, 0, 2*vessel_outer_radius+radius-0.01]  # Posizione del centro fisso
# Parametri dell'ago
needle_position = [0, 0, 2*vessel_outer_radius]  # Posizione iniziale
needle_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Orientamento iniziale
needle_id = create_needle_from_stl("ago_66mm.stl", needle_position, needle_orientation, scale=0.01)


# Posizionamento del vaso
vessel1 = p.loadSoftBody(
    "Arteria_piena.obj",
    simFileName="Arteria_piena.vtk",
    basePosition=[0, 0, 0],  # Centro del vaso allineato sull'asse Z
    baseOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    scale=0.1,
    mass=4,
    useNeoHookean=1,
    NeoHookeanMu=5000,
    NeoHookeanLambda=1000,
    NeoHookeanDamping=0.01,
    useSelfCollision=1,
    frictionCoeff=0.5,
    collisionMargin=0.01
)

# Slider per controllare velocità di rotazione e configurazione della telecamera
rotation_speed_slider = p.addUserDebugParameter("Rotation Speed", -5.0, 5.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("Camera Yaw", 0, 360, 145)
camera_pitch_slider = p.addUserDebugParameter("Camera Pitch", -180, 180, -5)
camera_distance_slider = p.addUserDebugParameter("Camera Distance", 0.1, 10, 2)

# Centro del semicerchio allineato con il centro del vaso
semicircle_center = [0, 0, 2*vessel_outer_radius]  # Centro fisso del semicerchio

# Ciclo di simulazione
while p.isConnected():
    # Simulazione
    p.stepSimulation()

    # Ottieni valori dagli sliders
    rotation_speed = p.readUserDebugParameter(rotation_speed_slider)
    yaw = p.readUserDebugParameter(camera_yaw_slider)
    pitch = p.readUserDebugParameter(camera_pitch_slider)
    distance = p.readUserDebugParameter(camera_distance_slider)

    # Aumenta l'angolo di rotazione
    angle += rotation_speed * (1 / 240)  # Incremento angolare


    # Calcola il nuovo orientamento (punta tangenzialmente alla traiettoria)
    needle_orientation = p.getQuaternionFromEuler([angle, 0, 0])

    # Aggiorna posizione e orientamento dell'ago
    p.resetBasePositionAndOrientation(needle_id, needle_center, needle_orientation)


    # Configura telecamera
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=semicircle_center
    )

    time.sleep(1 / 240)
