# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import csv
import time

from zmqRemoteApi import RemoteAPIClient
import numpy as np
import matplotlib.pyplot as plt

from numpy import pi, sin

np.set_printoptions(precision=3, suppress=True)


class BoaLink:
    def __init__(self, sim, link_sim_id: int):
        assert link_sim_id >= 0
        self.sim_id = link_sim_id
        self.sim = sim


class BoaJoint:
    def __init__(self, sim, joint_id: int, sensor_id: int, accelerometer_id: int):
        assert joint_id >= 0 and sensor_id >= 0
        self.joint_sim_id = joint_id
        self.sensor_sim_id = sensor_id
        self.accelerometer_id = accelerometer_id
        self.sim = sim

    def setJointVelocity(self, velocity: float):
        self.sim.setJointTargetVelocity(self.joint_sim_id, velocity)

    def setJointAngle(self, angle: float):
        self.sim.setJointTargetPosition(self.joint_sim_id, angle)

    def getForceSensor(self):
        _ret, force = self.sim.readForceSensor(self.sensor_sim_id)
        return np.array(force)[0:2]

    def getAccelerometer(self):
        _ret, force, _ = self.sim.readForceSensor(self.accelerometer_id)
        return np.array(force)[0:2] / 0.001

    def getJointAngle(self):
        return self.sim.getJointPosition(self.joint_sim_id)


class Boa:

    def __init__(self, sim, n_joints):
        object_ids = sim.getObjectsInTree(
            sim.getObject("/SnakeLink_0")
        )

        alias_to_ids = {sim.getObjectAlias(id): id for id in object_ids}
        print(alias_to_ids)

        joint_ids = [
            alias_to_ids[f"SnakeJoint_{i}"] for i in range(n_joints)
        ]
        sensor_ids = [
            alias_to_ids[f"ForceSensor_{i}"] for i in range(n_joints)
        ]
        accelerometer_ids = [
            alias_to_ids[f"AccelerometerSensor_{i}"] for i in range(n_joints)
        ]

        self.joints = [
            BoaJoint(sim, *i)
            for i in zip(joint_ids, sensor_ids, accelerometer_ids)
        ]

    def getJointAngles(self):
        return np.array([joint.getJointAngle() for joint in self.joints])

    def getForceSensors(self):
        return np.array([joint.getForceSensor() for joint in self.joints])

    def getAccelerometers(self):
        return np.array([joint.getAccelerometer() for joint in self.joints])


try:
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    boa = Boa(sim, n_joints=4)
    # When simulation is not running, ZMQ message handling could be a bit
    # slow, since the idle loop runs at 8 Hz by default. Make
    # sure that the idle loop runs at full speed for this program:
    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)

    # Fetch the ID of the obstacle sensor
    object_ids = sim.getObjectsInTree(
        sim.getObject("/Floor")
    )

    alias_to_ids = {sim.getObjectAlias(id): id for id in object_ids}
    obstacle_sensor = alias_to_ids["ObstacleSensor"]

    client.setStepping(True)
    sim.startSimulation()

    t = 0
    while t < 5:
        t = sim.getSimulationTime()
        for i, joint in enumerate(boa.joints):
            angle = pi/3 * sin(t + i*pi/2)
            joint.setJointAngle(angle)

        forces.append(boa.joints[3].getForceSensor()[0][1])
        accelerations.append(boa.joints[3].getAccelerometer()[1])

        _, tissemann, _ = sim.readForceSensor(obstacle_sensor)
        print(tissemann[0:2])
        tissemann = np.sqrt(tissemann[0]**2 + tissemann[1]**2)
        tissemenn.append(-tissemann)
        client.step()
        t += dt


finally:
    sim.stopSimulation()
    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

    print('Program ended')

    res1 = [force + 2*acc for force, acc in zip(forces, accelerations)]
    res2 = [force - 2*acc for force, acc in zip(forces, accelerations)]
    plt.plot(list(zip(res1, res2, tissemenn)))
    plt.show()
