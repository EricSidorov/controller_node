#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from numpy import zeros,average
from math import sin,sqrt,pi
from Abstractions import *
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState

class EffortPositionController(Controller):
    """PID position controller + feed-forward effort. k_eff=0: Pure PID, k_eff=1: Effort control"""
    required_prm = ['P','I','D']
    def __init__(self, params):
        super(EffortPositionController, self).__init__(params)
        self._ref = {'pos':0.0,'eff':0.0,'k_eff':0.0}
        self._I = 0.0
        self._last_err = 0.0
        self._last_t = rospy.Time(0)
    def reset(self):
        self._I = 0.0
        self._last_err = 0.0
        self._last_t = rospy.Time()
    def update(self,cur_state):
        # dt = (cur_state['Time'] - self._last_t).to_nsec()/1e9
        dt = (cur_state['Time'] - self._last_t).to_sec()
        err = self._ref['pos']-cur_state['pos']
        self._I += err*dt
        D = float(err - self._last_err)/float(dt)
        # print 'dt = ', dt,'E = ',err, 'D = ', D, 'I = ', self._I
        cmd = (float(self._params['P'])*float(err) + self._params['I']*self._I + float(self._params['D'])*float(D))*(1.0-self._ref['k_eff']) + self._ref['eff']*self._ref['k_eff']
        self._last_t = cur_state['Time']
        self._last_err = err
        return cmd

class EffortJointInterface(JointInterface):
    def __init__(self,topic_name):
        super(EffortJointInterface, self).__init__()
        self._pub = rospy.Publisher(topic_name, Float64)
    def set_command(self,comm):
        try:
            self._pub.publish(Float64(comm))
        except rospy.ROSException:
            pass
        

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
                if ros_controller_params[key2]['type'] == 'effort_controllers/JointEffortController':
                    if ros_controller_params[key2]['joint'] == jnt:
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
        def __init__(self, arg = []):    
            rospy.init_node('MultiJointController')
            rospy.sleep(0.1)

            self.CON = MultiEffPosCon()
            self.CON.load('/tmj')
            self._last_t = rospy.Time(0)
            self.JointsState = 0
            self.StartTime = 0
            self.StatePos = []
            self.StateVel = []
            self.StatePosDiff = []
            self.StateVelDiff = []
            self.MaxPoints = 10

            # self.ModelStatePub = rospy.Publisher('/gazebo/set_model_state',ModelState)
            # self.LinkStatePub = rospy.Publisher('/gazebo/set_link_state',LinkState)
            # self.PubRate = 1000.0;
            # rospy.sleep(0.1)

        def on_state_update(self,msg):
            if (msg.header.stamp - self._last_t)>=rospy.Duration(0.001):
                # print 'delay = ', (msg.header.stamp-rospy.Time.now())
                state = ParseJointStateMsg(msg)
                self.CON.update(state)
                self._last_t = msg.header.stamp
                self.JointsState = msg

                # if self.ControlMode == 0:
                #     self.CON.set_ref({'pole_axis':{'pos':0,'eff':self.Amp*sin(self.Freq*(rospy.Time.now().to_sec()-self.StartTime)),'k_eff':1.0}})
                # else:
                #     if rospy.Time.now().to_sec()-self.StartTime>1:
                #         self.StartTime = rospy.Time.now().to_sec()
                #         # if self.ref_pos>=2*pi:
                #         #     self.ref_pos = 0
                #         #     self.ref_dir = -1
                #         # if self.ref_pos<=-2*pi:
                #         #     self.ref_dir = 1
                #         # self.ref_pos = self.ref_pos + self.ref_dir*pi/30.0

                #         self.CON.set_ref({'pole_axis':{'pos':self.ref_pos,'eff':0,'k_eff':0.0}})

        def foot_contact_cb(self,msg):
            if self.Contact == 0 and len(msg.states)>4:
                if self.Reflex == 1:
                    self.CON.set_ref({'spring_axis':{'pos':self.spring0+self.piston_limit[1],'eff':0,'k_eff':0.0}})

                self.Contact = 1
                print "Touch down - Contacts: ",len(msg.states)
            if self.Contact == 1 and len(msg.states) == 0:
                if self.Reflex == 1:
                    self.CON.set_ref({'spring_axis':{'pos':self.spring0+self.piston_limit[0],'eff':0,'k_eff':0.0}})

                self.Contact = 0
                self.SectionCross()
                print "Lift off"


        def pid_param_update(self):
            self.CON._joints['spring_axis'].set_params({'P': self.spring_k, 'I': 0.0, 'D': self.spring_d})


        def Initialize(self):
            for i in range(10):
                self.CON.set_ref({'spring_axis':{'pos':0,'eff':0,'k_eff':1.0}})
                rospy.sleep(0.01)

            for i in range(100):
                self.CON.set_ref({'spring_axis':{'pos':0.05+i/100.0*(self.spring0-0.05),'eff':0,'k_eff':0.0}})
                rospy.sleep(0.01)


        def Jump(self,Times):
            Temp = self.Reflex
            self.Reflex = 0
            if Times>0:
                for i in range(Times):
                    self.CON.set_ref({'spring_axis':{'pos':self.spring0+self.piston_limit[0],'eff':0,'k_eff':0.0}})
                    rospy.sleep((1-self.CPGphase0)/self.CPGfreq)
                    self.CON.set_ref({'spring_axis':{'pos':self.spring0+self.piston_limit[1],'eff':0,'k_eff':0.0}})
                    rospy.sleep(self.CPGphase0/self.CPGfreq)
            else:
                while True:
                    self.CON.set_ref({'spring_axis':{'pos':self.spring0+self.piston_limit[0],'eff':0,'k_eff':0.0}})
                    rospy.sleep((1-self.CPGphase0)/self.CPGfreq)
                    self.CON.set_ref({'spring_axis':{'pos':self.spring0+self.piston_limit[1],'eff':0,'k_eff':0.0}})
                    rospy.sleep(self.CPGphase0/self.CPGfreq)
                    print self.JointsState
            self.Reflex = Temp
            print "Finished forced jumping"


        def SectionCross(self):
            # Save new crossing state
            self.StatePos.append(self.JointsState.position[0])
            self.StateVel.append(self.JointsState.velocity[0])

            N = len(self.StatePos)
            if N > 0:
                self.StatePosDiff.append(self.JointsState.position[0]-self.StatePos[N-2])
                self.StateVelDiff.append(self.JointsState.velocity[0]-self.StateVel[N-2])



            if N > self.MaxPoints:
                self.StatePos.pop(0)
                self.StateVel.pop(0)
                self.StatePosDiff.pop(0)
                self.StateVelDiff.pop(0)

                # Calculate average
                print 'Pos: ',average(self.StatePos),' | Vel: ',average(self.StateVel)

                # Calculate change
                print 'Diff. Pos: ',average(self.StatePosDiff),' | Vel: ',average(self.StateVelDiff)


        # def ResetToPose(self,Pose,dur):
        #     State = ModelState()
        #     State.model_name = 'gcb'
        #     State.pose.position.x = 0
        #     State.pose.position.y = 0
        #     State.pose.position.z = 1.01+dur/2.0
        #     State.reference_frame = 'ground_plane::link'

        #     LState = LinkState()
        #     LState.link_name = 'gcb::left_talus'
        #     LState.pose.position.x = 0
        #     LState.pose.position.y = 0
        #     LState.pose.position.z = 0.142 #+dur/2.0
        #     LState.reference_frame = 'ground_plane::link'

        #     self.CON.set_ref({'hip':{'pos':Pose[0],'eff':0,'k_eff':0.0},
        #                       'left_ankle':{'pos':Pose[1],'eff':0,'k_eff':0.0},
        #                       'right_ankle':{'pos':Pose[2],'eff':0,'k_eff':0.0}})

        #     rospy.sleep(0.1)
        #     ResetSteps = 0.3*dur*self.PubRate
        #     for i in range(int(ResetSteps)):
        #         self.ModelStatePub.publish(State)
        #         State.pose.position.z = 1.01+(ResetSteps-i)/ResetSteps*dur/2.0
        #         rospy.sleep(1.0/self.PubRate)
        #     # rospy.sleep(0.01)
        #     # self.ModelStatePub.publish(State)
        #     PoseError = 1
        #     Count = 0
        #     Hold = 1
        #     while Hold:
        #         self.LinkStatePub.publish(LState)
        #         # self.ModelStatePub.publish(State)
        #         rospy.sleep(1.0/self.PubRate)
        #         PoseError = (self.JointsState.position[0]-Pose[0])**2+(self.JointsState.position[1]-Pose[1])**2+(self.JointsState.position[2]-Pose[2])**2
                
        #         if PoseError<0.001:
        #             Count+=1
        #             # LState.pose.position.z = 0.142+(dur*self.PubRate-Count)*dur/2.0
        #             if Count>0.5*dur*self.PubRate:
        #                 Hold = 0
        #         else:
        #             Count = 0

        #     rospy.sleep(0.2*dur)


    eff_pos_node = EffPosNode()
    eff_pos_node.Contact = 0
    eff_pos_node.piston_limit=[0.0,0.11]
    eff_pos_node.spring0 = 0.7
    eff_pos_node.spring_k = 750
    eff_pos_node.spring_d = 10
    eff_pos_node.CPGfreq = 1.5
    eff_pos_node.CPGphase0 = 0.15
    rospy.Subscriber("/tmj/joint_states", JointState, eff_pos_node.on_state_update)
    eff_pos_node.pid_param_update()

    eff_pos_node.Initialize()
    eff_pos_node.Reflex = 1
    rospy.Subscriber('/foot_contact',ContactsState, eff_pos_node.foot_contact_cb)
    # Finished initializing the controller

    eff_pos_node.Jump(5)

    rospy.spin()


