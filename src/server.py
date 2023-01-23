import struct
from math import pi

from zmqRemoteApi import RemoteAPIClient


class sim():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.handles = {}

    def get_handles(self):
        labels = [
            'rollingJoint_rr', 'rollingJoint_rl', 'rollingJoint_fr', 'rollingJoint_fl',
            'youBot', 'ME_Platfo2_sub1', 'youBot_positionTarget', 'youBot_orientationBase',
            'kinect_rgb', 'kinect_depth', 'fastHokuyo_sensor1'
            ]

        for i in labels:
            self.handles[i] = self.sim.getObjectHandle(i)

    def step_enable(self):
        self.client.setStepping(True)

    def step_trigger(self):
        self.client.step()

    def start_sim(self):
        self.sim.startSimulation()
        for i in range(3):
            self.step_trigger()

    def stop_sim(self):
        self.sim.stopSimulation()

    def load_scene(self, name):
        self.sim.loadScene(name)

    def load_model(self, name):
        self.sim.loadModel(name)
        self.get_handles()

    def replace_robot(self, x, y, z):
        self.sim.setObjectPosition(self.handles['youBot'], -1,[x, y, z])

    def move(self, speed, rotVel):
        self.sim.setJointTargetVelocity(self.handles['rollingJoint_rr'], int(speed) + int(rotVel))
        self.sim.setJointTargetVelocity(self.handles['rollingJoint_rl'], int(speed) - int(rotVel))
        self.sim.setJointTargetVelocity(self.handles['rollingJoint_fr'], int(speed) + int(rotVel))
        self.sim.setJointTargetVelocity(self.handles['rollingJoint_fl'], int(speed) - int(rotVel))

    def get_robot_position(self):
        position = self.sim.getObjectPosition(self.handles['youBot'], -1)
        return position

    def get_robot_rotation(self):
        rot = self.sim.getObjectOrientation(self.handles['youBot_positionTarget'], -1)[2]
        return rot

    def turn(self, t_angle, speed):
        target_angle = self.get_robot_rotation() * 180.0 / pi - t_angle

        while abs(self.get_robot_rotation() * 180.0 / pi - target_angle) > 2.0:
            if t_angle >= 0.0:
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rr'], int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rl'], -int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fr'], int(speed) )
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fl'], -int(speed))
            else:
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rr'], -int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rl'], int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fr'], -int(speed) )
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fl'], int(speed))
            self.step_trigger()

    def get_lidar_data(self):
        ranges = self.sim.getStringSignal('scan ranges')
        lidar = [
            ray[0]
            for ray in struct.iter_unpack('<f', ranges)
            ]
        
        return lidar

    def get_lidar_inv(self):
        ranges = self.sim.getStringSignal('scan ranges')
        lidar = [
            ray[0] if ray[0] != 0.0 else 1.5
            for ray in struct.iter_unpack('<f', ranges)
            ]
            
        return lidar