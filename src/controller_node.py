#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class Controller(object):
	"""docstring for JointController"""
	def __init__(self, params):
		self._params = params
		self._setpoint = {};
	def reset(self):
		pass
	def update(self,cur_state):
		pass
	def set_ref(self,ref):
		self._setpoint.update(ref)
	def set_params(self,prm):
		self._params.update(prm)

class JointInterface(object):
	"""docstring for JointInterface"""
	def __init__(self):
		super(JointInterface, self).__init__()
	def send_command(self,comm):
		pass

class JointController(controller)

class EffortPositionController(Controller):
	"""docstring for EffortPositionController"""
	def __init__(self, params = {'P':100,'I':0.01,'D':10, 'k_eff' : 0.0}):
		super(EffortPositionController, self).__init__()
		self._setpoint = {'pos':0.0,'eff':0.0}
		self._I = 0.0
		self._last_err = 0.0
		self._last_t = rospy.Time(0)
		# self._publisher = rospy.Publisher(topic)
	def reset(self):
		self._I = 0.0
		self._last_err = 0.0
		self._last_t = rospy.Time.now()
	def update(self,cur_state):
		dt = rospy.Time.now() - self._last_t
		err = self._setpoint['pos']-cur_state
		self._I += err*dt
		D = (err - self._last_err)/dt
		cmd = (self._params['P']*err + self._params['I']*self._I + self._params['D']*D)*(1-self._params['k_eff']) + self._setpoint['eff']*self._params['k_eff']
		# self._publisher.publish(Float64(cmd))
		self._last_t = rospy.Time.now()
		self._last_err = err
		return cmd


		
class RobotController(object):
	def __init__(self,controller_type):
		self._joints = [] 
		if controller_type.__class__ == Controller:
			self._controller_type = controller_type
		else:
			raise TypeError('controller must be of class Controller')

	def load(self,joint_dict):
		# rospy.init_node(node_name)
		# namespace = '/'+robot_name
		# controller_params = rospy.get_param(namespace)
		for key in controller_params:
			params = {'P':100,'I':0.01,'D':10,'k_eff' = 0.0}
			con = controller_params[key]
			if ('type' in con) and (con['type'] == 'effort_controllers/JointEffortController'):
				joint = con['joint']
				if 'c_node_params' in con:
					params.update(con['c_node_params'])
			topic = '/'+key+'/'+'command'
			self._joints.append({'cotroller':self._controller_type(params),'interface':JointInterface(name = joint, comm_topic = topic)})

	def set_ref(self,comm):
		pass
	def set_params(self,params):
		pass
	def update(self,state):
		for jnt in self._joints:
			if jnt['interface'].name in state:
				comm = jnt['cotroller'].update(state[jnt['interface'].name])
				jnt['interface'].


		pass
		








