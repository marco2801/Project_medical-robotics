import pybullet as p
import pybullet_data
import numpy as np

# Connessione a PyBullet in modalità GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Crea il piano di base per la simulazione
plane_id = p.loadURDF("plane.urdf")

# Funzione per creare un cilindro per rappresentare i vasi
def create_vessel(radius, length, position, orientation):
    # Definizione della forma di collisione e visuale
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=radius, length=length,
                                          rgbaColor=[0.8, 0.3, 0.3, 1.0], specularColor=[0.5, 0.5, 0.5])
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=radius, height=length)
    # Creazione del cilindro come corpo rigido
    vessel_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape_id,
                                  baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                  baseOrientation=orientation)
    return vessel_id

# Parametri dei vasi
vessel_radius = 0.05      # Raggio dei vasi (5 cm)
vessel_length = 0.3       # Lunghezza dei vasi (30 cm)
gap = 0.02               # Spazio di 2 mm tra le estremità dei vasi

# Posizione e orientamento dei vasi per rappresentare l'anastomosi
# Posizioniamo i vasi lungo l'asse X con uno spazio di 2 mm tra le estremità
position_vessel1 = [-vessel_length / 2 - gap / 2, 0, vessel_radius]
position_vessel2 = [vessel_length / 2 + gap / 2, 0, vessel_radius]
orientation_vessel = p.getQuaternionFromEuler([0, np.pi / 2, 0])  # Orientamento orizzontale

# Creazione dei due vasi
vessel1_id = create_vessel(vessel_radius, vessel_length, position_vessel1, orientation_vessel)
vessel2_id = create_vessel(vessel_radius, vessel_length, position_vessel2, orientation_vessel)

# Configurazione della telecamera per una visualizzazione migliore
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=45, cameraPitch=-30,
                             cameraTargetPosition=[0, 0, vessel_radius])

# Mantieni la finestra aperta per osservare i vasi
print("Simulation running. Close the window to end.")
while p.isConnected():
    p.stepSimulation()
