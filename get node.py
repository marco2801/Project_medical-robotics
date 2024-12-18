import pybullet as p
import pybullet_data
import numpy as np
import time
from pathlib import Path

# Connessione a PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, -9.81)

BASE_DIR = Path(__file__).resolve().parent  # Cartella dello script
ASSETS_DIR = BASE_DIR / "Important file"  # Cartella contenente i file

# File di input
STL_FILE = str(ASSETS_DIR / "ago_66mm.stl")  # Converti in stringa per PyBullet
VESSEL_OBJ = str(ASSETS_DIR / "Arteria_piena.obj")
VESSEL_VTK = str(ASSETS_DIR / "Arteria_piena.vtk")

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

# Parametri dell'ago
needle_position = [0, 0, 2 * vessel_outer_radius]  # Posizione iniziale
needle_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])  # Orientamento iniziale
needle_id = create_needle_from_stl(STL_FILE, needle_position, needle_orientation, scale=0.01)

suture_angle=2*np.pi/6
# Caricamento dei vasi (soft body)
vessel1 = p.loadSoftBody(
    VESSEL_OBJ,
    simFileName=VESSEL_VTK,
    basePosition=[-vessel_outer_radius, -0.025, 0],
    baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]),
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

# Definisci la posizione target
vessel_outer_radius = 0.5  # Cambia con il valore effettivo
target_position = [vessel_outer_radius*np.sin(suture_angle),0,vessel_outer_radius+vessel_outer_radius*np.cos(suture_angle)]  # Posizione target

# Ottieni i dati della mesh del soft body
mesh_data_vessel1 = p.getMeshData(vessel1, flags=p.MESH_DATA_SIMULATION_MESH)

# Estraggo le posizioni dei nodi (lista di tuple XYZ)
node_positions_vessel1 = mesh_data_vessel1[1]  # Posizioni dei nodi
print(node_positions_vessel1)
# Trova il nodo più vicino al target_position
closest_node_index = None
min_distance = float('inf')

for i, pos in enumerate(node_positions_vessel1):
    # Calcola la distanza euclidea tra il nodo e la posizione target
    distance = np.linalg.norm(np.array(pos) - np.array(target_position))
    print(f"Distanza del nodo {i} (posizione {pos}) dalla posizione target {target_position}: {distance:.2f}")
    if distance < min_distance:
        min_distance = distance
        closest_node_index = i

# Mostra il risultato
if closest_node_index is not None:
    print(f"Nodo più vicino alla posizione {target_position}:")
    print(f"Indice del nodo: {closest_node_index}")
    print(f"Posizione del nodo: {node_positions_vessel1[closest_node_index]}")
    print(f"Distanza: {min_distance:.2f}")
else:
    print("Nessun nodo trovato.")

# (Facoltativo) Aggiungi una sfera visibile per evidenziare il nodo trovato
marker_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 0, 0, 1])
marker_body = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,  # Nessuna collisione
    baseVisualShapeIndex=marker_visual,
    basePosition=node_positions_vessel1[closest_node_index]
)
