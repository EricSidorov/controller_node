#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from Abstractions import *
from sensor_msgs.msg import JointState
class EffortPositionController(Controller):
    """PID position controller + feed-forward effort. k_eff=0: Pure PID, k_eff=1: Effort control"""
    required_prm = ['P','I','D','k_eff']
    def __init__(self, params):
        super(EffortPositionController, self).__init__(params)
        self._ref = {'pos':0.0,'eff':0.0}
        self._I = 0.0
        self._last_err = 0.0
        self._last_t = rospy.Time(0)
    def reset(self):
        self._I = 0.0
        self._last_err = 0.0
        self._last_t = rospy.Time()
    def update(self,cur_state):
        dt = (cur_state['Time'] - self._last_t).to_nsec()
        err = self._ref['pos']-cur_state['pos']
        self._I += err*dt
        D = float(err - self._last_err)/float(dt)
        # print 'dt = ', dt,'E = ',err, 'D = ', D, 'I = ', self._I
        cmd = (float(self._params['P'])*float(err) + self._params['I']*self._I + float(self._params['D'])*float(D))*(1.0-self._params['k_eff']) + self._ref['eff']*self._params['k_eff']
        self._last_t = cur_state['Time']
        self._last_err = err
        return cmd

class EffortJointInterface(JointInterface):
    def __init__(self,topic_name):
        super(EffortJointInterface, self).__init__()
        self._pub = rospy.Publisher(topic_name, Float64)
    def set_command(self,comm):
        self._pub.publish(Float64(comm))

class MultiEffPosCon(MultiJointController):
    """docstring for ROSRobotController"""
    def __init__(self):
        super(MultiEffPosCon, self).__init__()
    def load(self,robot_namespace):
        ros_controller_params = rospy.get_param(robot_namespace)
        effposcon_params = rospy.get_param('EffPosCon')
        for key1 in effposcon_params:
            jnt = key1
            params = effposcon_params[key1]['params']
            controller = EffortPositionController(params)
            eff_con_topic = None
            for key2 in ros_controller_params:
                if (ros_controller_params[key2]['joint'] == jnt) and (ros_controller_params[key2]['type'] == 'effort_controllers/JointEffortController'):
                    eff_con_topic = robot_namespace + '/' + key2 + '/command'
                    break
            if eff_con_topic == None:
                rospy.logwarn('Failed adding %s to EffPosCon: joint does not have an effort controller', jnt)
                continue
            else:
                interface = EffortJointInterface(eff_con_topic)
                self.add_joint(JointController(jnt_name = jnt, controller = controller, interface = interface))

    def add_joint(self,joint):
        super(MultiEffPosCon,self).add_joint(joint)
        rospy.loginfo('Loaded EffPosCon on joint %s', joint.get_name())

###########################

def ParseJointStateMsg(msg):
    d = dict({})
    for k,name in enumerate(msg.name):
        d.update({name:{'pos':msg.position[k], 'Time':rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs)}})
    return d

if __name__ == '__main__':
    class EffPosNode(object):
        def __init__(self):    
            rospy.init_node('MultiJointController')
            self.CON = MultiEffPosCon()
            self.CON.load('/gcb')
            self._last_t = rospy.Time(0)
        def on_state_update(self,msg):
            if (msg.header.stamp - self._last_t)>=rospy.Duration(0.001):
                state = ParseJointStateMsg(msg)
                self.CON.update(state)
                self._last_t = msg.header.stamp

    eff_pos_node = EffPosNode()
    rospy.Subscriber("/gcb/joint_states", JointState, eff_pos_node.on_state_update)
    rospy.spin()


