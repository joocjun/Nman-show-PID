import numpy as np


class PDControllerStable(object):
    """
    Implementation based on: Tan, J., Liu, K., & Turk, G. (2011). "Stable proportional-derivative controllers"
    DOI: 10.1109/MCG.2011.30
    """

    def __init__(self, pb):
        self._pb = pb

    def computePD(
        self,
        bodyUniqueId,
        jointIndices,
        desiredPositions,
        desiredVelocities,
        kps,
        kds,
        maxForces,
        timeStep,
    ):
        numJoints = self._pb.getNumJoints(bodyUniqueId)
        jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
        q1 = []
        qdot1 = []
        zeroAccelerations = []
        for i in range(numJoints):
            q1.append(jointStates[i][0])
            qdot1.append(jointStates[i][1])
            zeroAccelerations.append(0)

        q = np.array(q1)
        qdot = np.array(qdot1)
        qdes = np.array(desiredPositions)
        qdotdes = np.array(desiredVelocities)

        qError = qdes - q
        qdotError = qdotdes - qdot

        Kp = np.diagflat(kps)
        Kd = np.diagflat(kds)

        # Compute -Kp(q + qdot - qdes)
        p_term = Kp.dot(qError - qdot * timeStep)
        # Compute -Kd(qdot - qdotdes)
        d_term = Kd.dot(qdotError)

        # Compute Inertia matrix M(q)
        M = self._pb.calculateMassMatrix(bodyUniqueId, q1)
        M = np.array(M)
        # Given: M(q) * qddot + C(q, qdot) = T_ext + T_int
        # Compute Coriolis and External (Gravitational) terms G = C - T_ext
        G = self._pb.calculateInverseDynamics(
            bodyUniqueId, q1, qdot1, zeroAccelerations
        )
        G = np.array(G)
        # -------------------------------
        # IMPLEMENTATION STARTS
        # -------------------------------

        # Given:
        #   q           : (n,) current joint positions
        #   qdot        : (n,) current joint velocities
        #   q_desired   : (n,) desired joint positions
        #   qdot_desired: (n,) desired joint velocities
        #   kps         : (n,) proportional gains
        #   kds         : (n,) derivative gains
        #   timeStep    : float, simulation time step
        #   M           : (n, n) joint-space mass matrix
        #   G           : (n,) Coriolis + gravity vector
        #   maxForces   : (n,) actuator torque limits

        # TODO 1: Compute estimated joint acceleration using implicit formulation
        # Solve the linear system:
        #     (M + Kd * timeStep) @ qddot = p_term + d_term - G
        # Hint: use np.linalg.solve(A, b) to compute qddot

        qddot = ...

        # TODO 2: Compute control torque that accounts for implicit damping
        # Equation:
        #     tau = p_term + d_term - Kd @ qddot * timeStep

        tau = ...

        # -------------------------------
        # IMPLEMENTATION ENDS
        # -------------------------------
        # Clip generalized forces to actuator limits
        maxF = np.array(maxForces)
        generalized_forces = np.clip(tau, -maxF, maxF)
        return generalized_forces
