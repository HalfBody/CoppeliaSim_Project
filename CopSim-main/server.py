from cmath import pi
import time
import sys
import struct
import os
sys.path.append('remote_api/')

from zmqRemoteApi import RemoteAPIClient


class sim():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.handles = {}


    def get_handles(self):
        labels = ['rollingJoint_rr', 'rollingJoint_rl', 'rollingJoint_fr',
                  'rollingJoint_fl', 'youBot_positionTarget',
                  'youBot_orientationBase', 'kinect_rgb', 'kinect_depth', 'youBot', 'ME_Platfo2_sub1', 'fastHokuyo_sensor1']

        for i in labels:
            self.handles[i] = self.sim.getObjectHandle(i)
        #return handles


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
        if(rotVel != 0):
            print('ДВИЖЕНИЕ с поворотом')
        else:
            print('ДВИЖЕНИЕ')
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
        angle = self.get_robot_rotation()
        angle = angle*180/pi
        target_angle = angle - t_angle

        while abs(self.get_robot_rotation()*180/pi - target_angle) > 2:
            print('ПОВОРОТ')
            print('turn.t_angle: ', t_angle)
            if t_angle > 0:
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rr'], int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rl'], -int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fr'], int(speed) )
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fl'], -int(speed))
            if t_angle < 0:
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rr'], -int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_rl'], int(speed))
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fr'], -int(speed) )
                self.sim.setJointTargetVelocity(self.handles['rollingJoint_fl'], int(speed))
            print(abs(self.get_robot_rotation()*180/pi - target_angle), target_angle)
            #print(self.get_robot_rotation()*180/3.14)
            self.step_trigger()

    def get_lidar_data(self):
        #print('запуск лидара на сервере')
        ranges = self.sim.getStringSignal('scan ranges')
        #print(len(ranges))
        #print(ranges)
        b=[]
        for i in range(int(len(ranges)/4)):
            b.append(struct.unpack('<f',ranges[4*i:4*(i+1)])[0])
        print('server.get_lidar_data len(b): ', len(b))
        #print(b)
        lidar = []
        for i in b:
            # if i == 0.0:
            #     i = 1.5
            # if i >= 1.5:
            #     i = 1.5
            lidar.append(i)
        
        return lidar
