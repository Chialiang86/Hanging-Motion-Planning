import pybullet as p


# Create pybullet GUI
physics_client_id = p.connect(p.GUI)
p.resetDebugVisualizerCamera(
    cameraDistance=0.3,
    cameraYaw=0,
    cameraPitch=0,
    cameraTargetPosition=[0.0, 0.0, 0.0]
)
p.resetSimulation()
p.setPhysicsEngineParameter(numSolverIterations=150)
sim_timestep = 1.0 / 240
p.setTimeStep(sim_timestep)
p.setGravity(0, 0, -9.8)

p.loadURDF('Hook136/base.urdf')


while True:
    # key callback
    keys = p.getKeyboardEvents()            
    if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        break