import pybullet as p
import time
import os
import pybullet_data

from pdControllerExplicit import PDControllerExplicit
from pdControllerStable import PDControllerStable

# Setup
useMaximalCoordinates = False
useRealTimeSim = False
timeStep = 0.001

# PyBullet connect
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Path to custom URDF
script_dir = os.path.dirname(os.path.abspath(__file__))
custom_urdf_path = os.path.join(script_dir, "../asset/pid_motor/urdf/pid_motor.urdf")

# Load 4 instances for comparison
start_positions = [[-0.3, 0, 0], [-0.1, 0, 0], [0.1, 0, 0], [0.3, 0, 0]]
robots = [p.loadURDF(custom_urdf_path, pos, baseOrientation=p.getQuaternionFromEuler([0, 0, 3.14]), useMaximalCoordinates=useMaximalCoordinates, useFixedBase=True)
          for pos in start_positions]
pole_explicit, pole_stable, pole_plugin, pole_constraint = robots

# Controllers
explicitPD = PDControllerExplicit(p)
stablePD = PDControllerStable(p)
plugin_id = p.loadPlugin("pdControlPlugin")

# Disable default motors
for robot_id in robots:
    p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=0, force=0)

p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)
p.setRealTimeSimulation(useRealTimeSim)

# UI sliders
timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.01, timeStep)
desiredPosId = p.addUserDebugParameter("desiredPosition", -1.57, 1.57, 0)
desiredVelId = p.addUserDebugParameter("desiredVelocity", -5.0, 5.0, 0)
kpId = p.addUserDebugParameter("Kp", 0, 100, 50)
kdId = p.addUserDebugParameter("Kd", 0, 0.5, 0.02)
maxForceId = p.addUserDebugParameter("Max Force", 0, 100, 10)

# Debug labels
p.addUserDebugText("Explicit PD", [0, 0, 0.1], [1, 1, 1], parentObjectUniqueId=pole_explicit, parentLinkIndex=0)
p.addUserDebugText("Stable PD", [0, 0, 0.1], [1, 1, 1], parentObjectUniqueId=pole_stable, parentLinkIndex=0)
p.addUserDebugText("Plugin PD", [0, 0, 0.1], [1, 1, 1], parentObjectUniqueId=pole_plugin, parentLinkIndex=0)
p.addUserDebugText("Constraint PD", [0, 0, 0.1], [1, 1, 1], parentObjectUniqueId=pole_constraint, parentLinkIndex=0)

# Main loop
while p.isConnected():
    timeStep = p.readUserDebugParameter(timeStepId)
    p.setTimeStep(timeStep)

    # Slider values
    pos = p.readUserDebugParameter(desiredPosId)
    vel = p.readUserDebugParameter(desiredVelId)
    kp = p.readUserDebugParameter(kpId)
    kd = p.readUserDebugParameter(kdId)
    max_force = p.readUserDebugParameter(maxForceId)

    # Explicit PD
    tau_exp = explicitPD.computePD(pole_explicit,
                                   jointIndices=[0],
                                   desiredPositions=[pos],
                                   desiredVelocities=[vel],
                                   kps=[kp],
                                   kds=[kd],
                                   maxForces=[max_force],
                                   timeStep=timeStep)
    p.setJointMotorControl2(pole_explicit, 0, controlMode=p.TORQUE_CONTROL, force=tau_exp[0])

    # Stable PD
    tau_stable = stablePD.computePD(pole_stable,
                                    jointIndices=[0],
                                    desiredPositions=[pos],
                                    desiredVelocities=[vel],
                                    kps=[kp],
                                    kds=[kd],
                                    maxForces=[max_force],
                                    timeStep=timeStep)
    p.setJointMotorControl2(pole_stable, 0, controlMode=p.TORQUE_CONTROL, force=tau_stable[0])

    # Plugin PD
    if plugin_id >= 0:
        p.setJointMotorControl2(pole_plugin, 0, controlMode=p.PD_CONTROL,
                                targetPosition=pos,
                                targetVelocity=vel,
                                force=max_force,
                                positionGain=kp,
                                velocityGain=kd)

    # Constraint-based PD
    p.setJointMotorControl2(pole_constraint, 0, controlMode=p.POSITION_CONTROL,
                            targetPosition=pos,
                            targetVelocity=vel,
                            positionGain=timeStep * (kp / 150.),
                            velocityGain=0.5,
                            force=max_force)

    if not useRealTimeSim:
        p.stepSimulation()
        time.sleep(timeStep)
