#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class Controller(object):
    required_prm = []
    """Takes a reference and returns a command"""
    def __init__(self, params):
        if self._check_prm(params):
            self._params = params
        self._ref = {}
    def reset(self):
        pass
    def update(self,state):
        pass
    def set_ref(self,ref):
        self._ref.update(ref)
    def set_params(self,prm):
        self._params.update(prm)
    def _check_prm(self,prm):
        # Check if the params are eligable for this controller
        if set(self.__class__.required_prm) == set(prm.keys()):
            # print self.__class__.__name__ + 'Params correct'
            return True
        raise KeyError(self.__class__.__name__+': Invalid parameters')
        return False

class JointInterface(object):
    """Abstract interface to a joint"""
    def __init__(self):
        super(JointInterface, self).__init__()
    def set_command(self,comm):
        pass



class JointController(Controller):
    """ Wraps a controller with a joint interface """
    def __init__(self, jnt_name, controller, interface):
        self._controller = controller
        self._interface = interface
        self._name = jnt_name
    def reset(self):
        self._controller.reset()
    def set_ref(self,ref):
        self._controller.set_ref(ref)
    def set_params(self,prm):
        self._controller.set_params(prm)
    def update(self,state):
        # print self.get_name()
        comm = self._controller.update(state)
        self._interface.set_command(comm)
    def get_name(self):
        return self._name
    def on_load(self):
        pass

class MultiJointController(object):
    def __init__(self):
        self._joints = {}
    def add_joint(self,joint):
        if joint.__class__ == JointController:
            self._joints.update({joint.get_name():joint})
        else:
            raise TypeError('Must supply a JointController instance')
    def set_ref(self,refs):
        #refs should be a dict with keys matching joint names
        for key in refs:
            self._joints[key].set_ref(refs[key])
    def set_params(self,params):
        #params should be a dict with keys matching joint names
        for key in params:
            self._joints[key].set_params(params[key])
    def update(self,states):
        #states should be a dict with keys matching joint names
        for key in states:
            self._joints[key].update(states[key])
