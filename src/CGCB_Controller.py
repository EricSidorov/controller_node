#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from numpy import zeros
from math import sin
from Abstractions import *
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState,LinkState,ContactsState
from nav_msgs.msg import Odometry

class EffortPositionController(Controller):
    """PID position controller + feed-forward effort. k_eff=0: Pure PID, k_eff=1: Effort control"""
    required_prm = ['P','I','D']
    def __init__(self, params):
        super(EffortPositionController, self).__init__(params)
        self._ref = {'pos':0.0,'eff':0.0,'k_eff':1.0}
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
        # print '( ',float(self._params['P']),' * ',float(err),' + ',self._params['I'],' * ',self._I,' + ',float(self._params['D']),' * ',float(D),' ) * ( ',1.0-self._ref['k_eff'],' ) + ',self._ref['eff'],' * ',self._ref['k_eff'],' = ',cmd
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

def ParseJointStateMsg(msg,Time):
    d = dict({})
    for k,name in enumerate(msg.name):
        # d.update({name:{'pos':msg.position[k], 'Time':rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs)}})
        d.update({name:{'pos':msg.position[k], 'Time':Time}})
    return d


if __name__ == '__main__':
    class EffPosNode(object):
        def __init__(self, arg = []):    
            rospy.init_node('MultiJointController')
            rospy.sleep(0.1)

            self.CON = MultiEffPosCon()
            self.CON.load('/cgcb')
            self._last_t = rospy.Time(0)
            self.JointsState = 0
            self.LeftContact = 0
            self.RightContact = 0
            self.GlobalPos = 0
            self.StartTime = 0

            self.ModelStatePub = rospy.Publisher('/gazebo/set_model_state',ModelState)
            self.LinkStatePub = rospy.Publisher('/gazebo/set_link_state',LinkState)
            self.PubRate = 1000.0;
            rospy.sleep(0.1)

            ##################################################################
            ######################## GAIT PARAMETERS #########################
            ##################################################################
            if len(arg)<12:
                self.SwingEff0 = 0.80        # Default swing effort
                self.kAp = 2.0               # Aperture feedback gain
                self.DesAp = 0.38            # Desired aperture
                self.SupAnkGain_p = 30       # Ankle gain p during support
                self.SupAnkGain_d = 5        # Ankle gain d during support
                self.SupAnkSetPoint = -0.05  # Support ankle set point
                self.ToeOffEff0 = 5.5        # Toe-off effort
                self.ToeOffDur = 0.13        # Toe-off duration
                self.TotSwingDur = 0.75      # Swing duration
                # self.FootExtEff = 0.65       # Foot extension effort (right before touch down)
                self.FootExtDur = 0.20       # Foot extension duration
                self.FreeSwingDur = 0.17     # Swing duration
                
                self.SwingEff = self.SwingEff0
                self.ToeOffEff = self.ToeOffEff0
            else:
                self.SwingEff0 = arg[0]      # Default swing effort
                self.kAp = arg[1]            # Aperture feedback gain
                self.DesAp = arg[2]          # Desired aperture
                self.SupAnkGain_p = arg[3]   # Ankle gain p during support
                self.SupAnkGain_d = arg[4]   # Ankle gain d during support
                self.SupAnkSetPoint = arg[5] # Support ankle set point
                self.ToeOffEff0 = arg[6]      # Toe-off effort
                self.ToeOffDur = arg[7]      # Toe-off duration
                self.TotSwingDur = arg[8]    # Swing duration
                # self.FootExtEff = arg[9]     # Foot extension effort (right before touch down)
                self.FootExtDur = arg[9]    # Foot extension duration
                self.FreeSwingDur = arg[10]  # Swing duration
                
                self.SwingEff = self.SwingEff0
                self.ToeOffEff = self.ToeOffEff0

        def on_state_update(self,msg):
            Now = rospy.Time.now()
            if (Now - self._last_t)>=rospy.Duration(0.001):
                # print (Now - self._last_t) / 1000000.0
                # print 'delay = ', (msg.header.stamp-rospy.Time.now()).to_sec()*1000.0
                # self.AvgDelay = self.AvgDelay+
                state = ParseJointStateMsg(msg,Now)
                # print state
                self.CON.update(state)
                self._last_t = Now
                self.JointsState = msg

                self.CON.set_ref({'planar_x':{'pos':0.5*sin(Now.to_sec()),'eff':0,'k_eff':0.0}})
                self.CON.set_ref({'planar_z':{'pos':0,'eff':0,'k_eff':0.0}})

                # 0 - hip
                # 1 - left_ankle
                # 2 - pelvis_joint
                # 3 - planar_x
                # 4 - planar_z
                # 5 - right_ankle


        def Odom_cb(self,msg):
            self.GlobalPos = msg.pose.pose.position

            # Check elapsed time
            self.TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

            if self.Go == 1 and (self.GlobalPos.z<0.6 or self.GlobalPos.z>1.4 or (self.TimeElapsed>self.TimeOut and self.TimeOut>0)):
                # if the robot fell, stop running and return fitness
                self.Go = 0


        # def left_contact_cb(self,msg):
        #     if self.LeftContact == 0 and len(msg.states)>4:
        #         self.LeftContact = 1
        #         print "Left touch down - Contacts: ",len(msg.states)
        #     if self.LeftContact == 1 and len(msg.states) == 0:
        #         self.LeftContact = 0
        #         # print "Left swing"

        # def right_contact_cb(self,msg):
        #     if self.RightContact == 0 and len(msg.states)>4:
        #         self.RightContact = 1
        #         print "Right touch down - Contacts: ",len(msg.states)
        #     if self.RightContact == 1 and len(msg.states) == 0:
        #         self.RightContact = 0
        #         # print "Right swing"


        def pid_param_update(self):
            self.CON._joints['planar_x'].set_params({'P': 200.0, 'I': 0.0, 'D': 20.0})
            # self.CON._joints['planar_z'].set_params({'P': 200.0, 'I': 0.0, 'D': 50.0})
            self.CON._joints['planar_z'].set_params({'P': 200.0, 'I': 0.0, 'D': 25.0})
            # self.CON._joints['pelvis_joint'].set_params({'P': 0.0, 'I': 0.0, 'D': 10.0})
            # self.CON._joints['hip'].set_params({'P': 120.0, 'I': 0.0, 'D': 10.0})
            # self.CON._joints['left_ankle'].set_params({'P': 60.0, 'I': 0.0, 'D': 5.0})
            # self.CON._joints['right_ankle'].set_params({'P': 60.0, 'I': 0.0, 'D': 5.0})


        # def ResetToPose(self,Pose,dur):
        #     State = ModelState()
        #     State.model_name = 'cgcb'
        #     State.pose.position.x = 0
        #     State.pose.position.y = 0
        #     State.pose.position.z = 0
        #     State.reference_frame = 'ground_plane::link'

        #     # LState = LinkState()
        #     # LState.link_name = 'cgcb::left_talus'
        #     # LState.pose.position.x = 0
        #     # LState.pose.position.y = 0
        #     # LState.pose.position.z = 0.142 #+dur/2.0
        #     # LState.reference_frame = 'ground_plane::link'
        #     rospy.sleep(0.05)
        #     dHeight = 1.0
               
        #     CurPose = zeros(6)
        #     CurPose[0] = self.JointsState.position[3] # 0 - planar_x
        #     CurPose[1] = self.JointsState.position[4] # 1 - planar_z
        #     CurPose[2] = self.JointsState.position[2] # 2 - pelvis_joint
        #     CurPose[3] = self.JointsState.position[0] # 3 - hip
        #     CurPose[4] = self.JointsState.position[1] # 4 - left_ankle
        #     CurPose[5] = self.JointsState.position[5] # 5 - right_ankle

        #     rospy.sleep(0.05)

        #     # Raise the robot
        #     Steps = 0.2*dur*self.PubRate
        #     for i in range(int(Steps)):
        #         Height = CurPose[1] + float(i)/Steps*dHeight
        #         self.CON.set_ref({'planar_x':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'planar_z':{'pos':Height,'eff':0,'k_eff':0.0},
        #                           'pelvis_joint':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':0,'k_eff':1.0}})
        #         rospy.sleep(1.0/self.PubRate)
        #     print "Finished step 1"
        #     self.ModelStatePub.publish(State)

        #     CurPose = zeros(6)
        #     CurPose[0] = self.JointsState.position[3] # 0 - planar_x
        #     CurPose[1] = self.JointsState.position[4] # 1 - planar_z
        #     CurPose[2] = self.JointsState.position[2] # 2 - pelvis_joint
        #     CurPose[3] = self.JointsState.position[0] # 3 - hip
        #     CurPose[4] = self.JointsState.position[1] # 4 - left_ankle
        #     CurPose[5] = self.JointsState.position[5] # 5 - right_ankle

        #     Height = Pose[1]
        #     Pose[1] = Pose[1] + dHeight

        #     Steps = 0.3*dur*self.PubRate
        #     for x in range(int(Steps)):
        #         i = float(x)/Steps
        #         RefPose = i*Pose+(1.0-i)*CurPose
        #         # RefPose = CurPose

        #         self.CON.set_ref({'planar_x':{'pos':RefPose[0],'eff':0,'k_eff':1.0},
        #                           'planar_z':{'pos':RefPose[1],'eff':0,'k_eff':0.0},
        #                           'pelvis_joint':{'pos':RefPose[2],'eff':0,'k_eff':1.0},
        #                           'hip':{'pos':RefPose[3],'eff':0,'k_eff':0.0},
        #                           'left_ankle':{'pos':RefPose[4],'eff':0,'k_eff':0.0},
        #                           'right_ankle':{'pos':RefPose[5],'eff':0,'k_eff':0.0}})
        #         # self.CON.set_ref({'planar_x':{'pos':RefPose[0],'eff':0,'k_eff':1.0},
        #         #                   'planar_z':{'pos':RefPose[1],'eff':0,'k_eff':1.0}})
        #         rospy.sleep(1.0/self.PubRate)

        #     print "Finished step 2"
        #     # Maybe add while here

        #     # PoseError = 1
        #     # Count = 0
        #     # Hold = 1
        #     # while Hold:
        #     #     # self.LinkStatePub.publish(LState)
        #     #     # self.ModelStatePub.publish(State)
        #     #     rospy.sleep(1.0/self.PubRate)
        #     #     PoseError = (self.JointsState.position[0]-Pose[0])**2+(self.JointsState.position[1]-Pose[1])**2+(self.JointsState.position[5]-Pose[2])**2
                
        #     #     if PoseError<0.001:
        #     #         Count+=1
        #     #         # LState.pose.position.z = 0.142+(dur*self.PubRate-Count)*dur/2.0
        #     #         if Count>0.5*dur*self.PubRate:
        #     #             Hold = 0
        #     #     else:
        #     #         Count = 0

        #     Steps = 0.5*dur*self.PubRate
        #     for i in range(int(Steps)):
        #         Pose[1] = Height - float(i)/Steps*dHeight
        #         self.CON.set_ref({'planar_x':{'pos':Pose[0],'eff':0,'k_eff':1.0},
        #                           'planar_z':{'pos':Pose[1],'eff':0,'k_eff':0.0},
        #                           'pelvis_joint':{'pos':Pose[2],'eff':0,'k_eff':1.0},
        #                           'hip':{'pos':Pose[3],'eff':0,'k_eff':0.0},
        #                           'left_ankle':{'pos':Pose[4],'eff':0,'k_eff':0.0},
        #                           'right_ankle':{'pos':Pose[5],'eff':0,'k_eff':0.0}})
        #         rospy.sleep(1.0/self.PubRate)

        #     # Maybe add sleep here

        #     # rospy.sleep(0.2*dur)


        # def TakeFirstStep(self,which):
        #     if which == "right":
        #         self.CON.set_ref({'hip':{'pos':0,'eff':-5.5,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':0.2,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':-3,'k_eff':1.0}})
        #         # self.JC.set_eff('right_ankle',-3)
        #         # self.JC.set_eff('hip',5.5)
        #         # self.JC.set_eff('left_ankle',0.2)
        #         # self.JC.send_command()
        #         rospy.sleep(0.1)
        #         self.CON.set_ref({'hip':{'pos':0,'eff':-5.5,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':0.2,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':0,'k_eff':1.0}})
        #         # self.JC.set_eff('right_ankle',0)
        #         # self.JC.send_command()
        #         rospy.sleep(0.55)
        #         self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':1,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':0,'k_eff':1.0}})
        #         # self.JC.set_eff('hip',0)
        #         # self.JC.set_eff('left_ankle',1)
        #         # self.JC.send_command()
        #         rospy.sleep(0.1)

        #     if which == "left":
        #         self.CON.set_ref({'hip':{'pos':0,'eff':5.5,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':-3,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':0.2,'k_eff':1.0}})
        #         # self.JC.set_eff('left_ankle',-3)
        #         # self.JC.set_eff('hip',-5.5)
        #         # self.JC.set_pos('right_ankle',0.2)
        #         # self.JC.send_command()
        #         rospy.sleep(0.1)
        #         self.CON.set_ref({'hip':{'pos':0,'eff':5.5,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':0.2,'k_eff':1.0}})
        #         # self.JC.set_eff('left_ankle',0)
        #         # self.JC.send_command()
        #         rospy.sleep(0.55)
        #         self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'left_ankle':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'right_ankle':{'pos':0,'eff':1,'k_eff':1.0}})
        #         # self.JC.set_eff('hip',0)
        #         # self.JC.set_eff('right_ankle',1)
        #         # self.JC.send_command()
        #         rospy.sleep(0.1)

        # def TakeStep(self,which):
        #     if self.TimeElapsed<60:
        #         Aperture = abs(self.JointsState.position[0])
        #         self.SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
        #         # self.ToeOffEff = self.ToeOffEff0 + self.kAp*(self.DesAp - Aperture)
                
        #         # print Aperture
        #     else:
        #         Aperture = abs(self.JointsState.position[0])
        #         self.SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
        #         #self.ToeOffEff = self.ToeOffEff0 - 2*(self.DesAp - Aperture)
        #         #print self.ToeOffEff
        #         #print self.SwingEff

        #     if which == "right":
        #         # Toe off
        #         self.CON._joints['left_ankle'].set_params({'P': self.SupAnkGain_p, 'I': 0.0, 'D': self.SupAnkGain_d})
        #         self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'right_ankle':{'pos':0,'eff':self.ToeOffEff,'k_eff':1.0}})
        #         # self.JC.set_gains('left_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
        #         # self.JC.set_pos('left_ankle',self.SupAnkSetPoint)
        #         # self.JC.set_eff('right_ankle',self.ToeOffEff)
        #         # self.JC.set_eff('hip',0)
        #         # self.JC.send_command()
        #         rospy.sleep(self.ToeOffDur)
        #         # Raise feet and swing
        #         self.CON._joints['right_ankle'].set_params({'P': 35, 'I': 0.0, 'D': 3})
        #         self.CON.set_ref({'hip':{'pos':0,'eff':-self.SwingEff,'k_eff':1.0},
        #                           'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'right_ankle':{'pos':-1.57,'eff':0,'k_eff':0.0}})
        #         # # self.JC.set_eff('right_ankle',-3)
        #         # self.JC.set_gains('right_ankle',30,15,0,set_default = False)
        #         # self.JC.set_pos('right_ankle',-1.57)
        #         # self.JC.set_eff('hip',self.SwingEff)
        #         # self.JC.send_command()
        #         rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
        #         # Lower feet
        #         self.CON.set_ref({'hip':{'pos':0,'eff':-self.SwingEff,'k_eff':1.0},
        #                           'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'right_ankle':{'pos':self.DesAp,'eff':0,'k_eff':0.0}})
        #         # self.JC.set_eff('right_ankle',self.FootExtEff)
        #         # self.JC.send_command()
        #         rospy.sleep(self.FootExtDur)
        #         # End swing
        #         self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'right_ankle':{'pos':self.DesAp,'eff':0,'k_eff':1.0}})
        #         # self.JC.set_eff('hip',0)
        #         # self.JC.set_eff('right_ankle',0)
        #         # self.JC.send_command()
        #         rospy.sleep(self.FreeSwingDur)

        #     if which == "left":
        #         # Toe off
        #         self.CON._joints['right_ankle'].set_params({'P': self.SupAnkGain_p, 'I': 0.0, 'D': self.SupAnkGain_d})
        #         self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'left_ankle':{'pos':0,'eff':self.ToeOffEff,'k_eff':1.0}})
        #         # self.JC.set_gains('right_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
        #         # self.JC.set_pos('right_ankle',self.SupAnkSetPoint)
        #         # self.JC.set_eff('left_ankle',self.ToeOffEff)
        #         # self.JC.set_eff('hip',0)
        #         # self.JC.send_command()
        #         rospy.sleep(self.ToeOffDur)
        #         # Raise feet and swing
        #         self.CON._joints['left_ankle'].set_params({'P': 35, 'I': 0.0, 'D': 3})
        #         self.CON.set_ref({'hip':{'pos':0,'eff':self.SwingEff,'k_eff':1.0},
        #                           'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'left_ankle':{'pos':-1.57,'eff':0,'k_eff':0.0}})
        #         # # self.JC.set_eff('right_ankle',-3)
        #         # self.JC.set_gains('left_ankle',30,15,0,set_default = False)
        #         # self.JC.set_pos('left_ankle',-1.57)
        #         # self.JC.set_eff('hip',self.SwingEff)
        #         # self.JC.send_command()
        #         rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
        #         # Lower feet
        #         self.CON.set_ref({'hip':{'pos':0,'eff':self.SwingEff,'k_eff':1.0},
        #                           'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'left_ankle':{'pos':self.DesAp,'eff':0,'k_eff':0.0}})
        #         # self.JC.set_eff('left_ankle',self.FootExtEff)
        #         # self.JC.send_command()
        #         rospy.sleep(self.FootExtDur)
        #         # End swing
        #         self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
        #                           'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
        #                           'left_ankle':{'pos':self.DesAp,'eff':0,'k_eff':1.0}})
        #         # self.JC.set_eff('hip',0)
        #         # self.JC.set_eff('left_ankle',0)
        #         # self.JC.send_command()
        #         rospy.sleep(self.FreeSwingDur)


    eff_pos_node = EffPosNode()
    eff_pos_node.TimeOut = 300
    eff_pos_node.Go=0
    rospy.Subscriber("/cgcb/joint_states", JointState, eff_pos_node.on_state_update)
    # rospy.Subscriber('/ground_truth_odom',Odometry, eff_pos_node.Odom_cb)
    # rospy.Subscriber('/left_foot_contact',ContactsState, eff_pos_node.left_contact_cb)
    # rospy.Subscriber('/right_foot_contact',ContactsState, eff_pos_node.right_contact_cb)
    eff_pos_node.pid_param_update()
    # Finished initializing the controller

    # Start with right foot raised
    InitPose = zeros(6)
    InitPose[0] = 0 # X
    InitPose[1] = 0 # Z
    InitPose[2] = 0 # pitch
    InitPose[3] = -0.1 # hip
    InitPose[4] = 0.06 # left_ankle
    InitPose[5] = -1.45 # right_ankle

    rospy.sleep(0.1)
    # eff_pos_node.ResetToPose(InitPose,10)
    # State = ModelState()
    # State.model_name = 'cgcb'
    # State.pose.position.x = 0
    # State.pose.position.y = 0
    # State.pose.position.z = 0
    # State.reference_frame = 'ground_plane::link'
    # LState1 = LinkState()
    # LState1.link_name = 'cgcb::pelvis'
    # LState1.pose.position.x = 0
    # LState1.pose.position.y = 0
    # LState1.pose.position.z = 1 #+dur/2.0
    # LState1.reference_frame = 'ground_plane::link'
    # LState2 = LinkState()
    # LState2.link_name = 'cgcb::left_leg'
    # LState2.pose.position.x = 0
    # LState2.pose.position.y = 0
    # LState2.pose.position.z = 1 #+dur/2.0
    # LState2.reference_frame = 'ground_plane::link'
    # LState3 = LinkState()
    # LState3.link_name = 'cgcb::right_leg'
    # LState3.pose.position.x = 0
    # LState3.pose.position.y = 0
    # LState3.pose.position.z = 1 #+dur/2.0
    # LState3.reference_frame = 'ground_plane::link'
    # rospy.sleep(0.1)
    # eff_pos_node.ModelStatePub.publish(State)
    # eff_pos_node.LinkStatePub.publish(LState1)
    # eff_pos_node.LinkStatePub.publish(LState2)
    # eff_pos_node.LinkStatePub.publish(LState3)
    # print "Done"

    # # Save start time
    # eff_pos_node.StartTime = rospy.Time.now().to_sec()

    # # Take first step
    # eff_pos_node.TakeFirstStep("right")
    # StepsTaken = 1

    # # Initialize aperture measurement
    # ApertureMean = abs(eff_pos_node.JointsState.position[0])
    # ApertureStd = 0

    # # Rinse and repeat
    # Leg = "left"
    # eff_pos_node.Go=1
    # while eff_pos_node.Go == 1:
    #     # Take another step
    #     eff_pos_node.TakeStep(Leg)
    #     StepsTaken += 1

    #     # ################################################################
    #     ThisAperture = abs(eff_pos_node.JointsState.position[0])
    #     ApertureMean += (ThisAperture-ApertureMean) / StepsTaken
    #     ApertureStd += ((ThisAperture-ApertureMean)**2 - ApertureStd) / StepsTaken

    #     if Leg == "right":
    #         Leg = "left"
    #     else:
    #         Leg = "right"

    # print "Runtime: ",eff_pos_node.TimeElapsed," - Steps taken: ",StepsTaken
    # print "Step size: ",2*sin(ApertureMean/2)," (+/-",2*sin(ApertureStd/2),")"

    rospy.spin()


