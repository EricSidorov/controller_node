#!/usr/bin/env python
from Abstractions import *
from numpy import array, zeros, linspace
import matplotlib.pyplot as plt
from copy import copy

#### Create objects ####

class DummyJoint(object):
    # A simple model of a mass
    def __init__(self,IC,M):
        super(DummyJoint, self).__init__()
        self._state = IC
        self._M = M
    def update(self,force_input):
        vel_old = copy(self._state[1])
        self._state[0]+=self._state[1]
        self._state[1]+=force_input/self._M-0.1*vel_old
    def get_state(self):
        return self._state

class DummyController(Controller):
    # simple proportional controller
    required_prm = ['K']
    def __init__(self,params):
        super(DummyController, self).__init__(params)
        self._ref = {'pos':0.0}
    def update(self,state):
        comm = self._params['K']*(self._ref['pos']-state)
        return comm


class DummyInterface(JointInterface):
    """docstring for DummyInterface"""
    def __init__(self,joint_input_func):
        super(DummyInterface, self).__init__()
        self._func = joint_input_func
    def set_command(self,comm):
        self._func(comm)

########### TEST ############

# create 3 dummy joints
JNT1 = DummyJoint([0,0],1)
JNT2 = DummyJoint([2,0],5)
JNT3 = DummyJoint([1,0],10)
#create RobotController
DummyRC = MultiJointController()
#Add 3 joint controllers
DummyRC.add_joint(JointController(jnt_name = 'joint_1', controller = DummyController({'K':0.01}), interface = DummyInterface(JNT1.update)))
DummyRC.add_joint(JointController(jnt_name = 'joint_2', controller = DummyController({'K':0.01}), interface = DummyInterface(JNT2.update)))
DummyRC.add_joint(JointController(jnt_name = 'joint_3', controller = DummyController({'K':0.01}), interface = DummyInterface(JNT3.update)))
#Run Test
N = 600
ks = array(range(0,N))
pos_1 = zeros(N)
pos_2 = zeros(N)
pos_3 = zeros(N)
DummyRC.set_params({'joint_3':{'K':0.5}})
for k in xrange(0,N-1):
    pos_1[k] = JNT1.get_state()[0]
    pos_2[k] = JNT2.get_state()[0]
    pos_3[k] = JNT3.get_state()[0]
    states = {'joint_1':pos_1[k],'joint_2':pos_2[k],'joint_3':pos_3[k]}
    DummyRC.update(states)

plt.plot(ks,pos_1,ks,pos_2,ks,pos_3)
plt.show()










        