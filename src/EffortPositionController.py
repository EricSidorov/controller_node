#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from numpy import zeros
from math import sin
from Abstractions import *
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from nav_msgs.msg import Odometry

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
        dt = (cur_state['Time'] - self._last_t).to_nsec()/1e9
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
        def __init__(self, arg = []):    
            rospy.init_node('MultiJointController')
            rospy.sleep(0.1)

            self.CON = MultiEffPosCon()
            self.CON.load('/gcb')
            self._last_t = rospy.Time(0)
            self.JointsState = 0
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
                self.SwingEff0 = 0.30        # Default swing effort
                self.kAp = 1.0               # Aperture feedback gain
                self.DesAp = 0.40            # Desired aperture
                self.SupAnkGain_p = 30       # Ankle gain p during support
                self.SupAnkGain_d = 5        # Ankle gain d during support
                self.SupAnkSetPoint = -0.05  # Support ankle set point
                self.ToeOffEff0 = 4.0        # Toe-off effort
                self.ToeOffDur = 0.22        # Toe-off duration
                self.TotSwingDur = 0.80      # Swing duration
                # self.FootExtEff = 0.65       # Foot extension effort (right before touch down)
                self.FootExtDur = 0.22       # Foot extension duration
                self.FreeSwingDur = 0.13     # Swing duration
                
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
            if (msg.header.stamp - self._last_t)>=rospy.Duration(0.001):
                state = ParseJointStateMsg(msg)
                self.CON.update(state)
                self._last_t = msg.header.stamp
                self.JointsState = msg

        def Odom_cb(self,msg):
            self.GlobalPos = msg.pose.pose.position

            # Check elapsed time
            self.TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

            if self.Go == 1 and (self.GlobalPos.z<0.6 or self.GlobalPos.z>1.4 or (self.TimeElapsed>self.TimeOut and self.TimeOut>0)):
                # if the robot fell, stop running and return fitness
                self.Go = 0


        def pid_param_update(self):
            self.CON._joints['hip'].set_params({'P': 120.0, 'I': 0.0, 'D': 10.0})
            self.CON._joints['left_ankle'].set_params({'P': 60.0, 'I': 0.0, 'D': 5.0})
            self.CON._joints['right_ankle'].set_params({'P': 60.0, 'I': 0.0, 'D': 5.0})


        def ResetToPose(self,Pose,dur):
            State = ModelState()
            State.model_name = 'gcb'
            State.pose.position.x = 0
            State.pose.position.y = 0
            State.pose.position.z = 1.01+dur/2.0
            State.reference_frame = 'ground_plane::link'

            LState = LinkState()
            LState.link_name = 'gcb::left_talus'
            LState.pose.position.x = 0
            LState.pose.position.y = 0
            LState.pose.position.z = 0.142 #+dur/2.0
            LState.reference_frame = 'ground_plane::link'

            self.CON.set_ref({'hip':{'pos':Pose[0],'eff':0,'k_eff':0.0},
                              'left_ankle':{'pos':Pose[1],'eff':0,'k_eff':0.0},
                              'right_ankle':{'pos':Pose[2],'eff':0,'k_eff':0.0}})

            rospy.sleep(0.1)
            ResetSteps = 0.3*dur*self.PubRate
            for i in range(int(ResetSteps)):
                self.ModelStatePub.publish(State)
                State.pose.position.z = 1.01+(ResetSteps-i)/ResetSteps*dur/2.0
                rospy.sleep(1.0/self.PubRate)
            # rospy.sleep(0.01)
            # self.ModelStatePub.publish(State)
            PoseError = 1
            Count = 0
            Hold = 1
            while Hold:
                self.LinkStatePub.publish(LState)
                # self.ModelStatePub.publish(State)
                rospy.sleep(1.0/self.PubRate)
                PoseError = (self.JointsState.position[0]-Pose[0])**2+(self.JointsState.position[1]-Pose[1])**2+(self.JointsState.position[2]-Pose[2])**2
                
                if PoseError<0.001:
                    Count+=1
                    # LState.pose.position.z = 0.142+(dur*self.PubRate-Count)*dur/2.0
                    if Count>0.5*dur*self.PubRate:
                        Hold = 0
                else:
                    Count = 0

            rospy.sleep(0.2*dur)

            # Duration = int(dur*self.PubRate)
            # for x in range(Duration):
            #     # self.ModelStatePub.publish(State)
            #     self.LinkStatePub.publish(LState)
            #     rospy.sleep(1.0/self.PubRate)

        def TakeFirstStep(self,which):
            if which == "right":
                self.CON.set_ref({'hip':{'pos':0,'eff':-5.5,'k_eff':1.0},
                                  'left_ankle':{'pos':0,'eff':0.2,'k_eff':1.0},
                                  'right_ankle':{'pos':0,'eff':-3,'k_eff':1.0}})
                # self.JC.set_eff('right_ankle',-3)
                # self.JC.set_eff('hip',5.5)
                # self.JC.set_eff('left_ankle',0.2)
                # self.JC.send_command()
                rospy.sleep(0.1)
                self.CON.set_ref({'hip':{'pos':0,'eff':-5.5,'k_eff':1.0},
                                  'left_ankle':{'pos':0,'eff':0.2,'k_eff':1.0},
                                  'right_ankle':{'pos':0,'eff':0,'k_eff':1.0}})
                # self.JC.set_eff('right_ankle',0)
                # self.JC.send_command()
                rospy.sleep(0.55)
                self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
                                  'left_ankle':{'pos':0,'eff':1,'k_eff':1.0},
                                  'right_ankle':{'pos':0,'eff':0,'k_eff':1.0}})
                # self.JC.set_eff('hip',0)
                # self.JC.set_eff('left_ankle',1)
                # self.JC.send_command()
                rospy.sleep(0.1)

            if which == "left":
                self.CON.set_ref({'hip':{'pos':0,'eff':5.5,'k_eff':1.0},
                                  'left_ankle':{'pos':0,'eff':-3,'k_eff':1.0},
                                  'right_ankle':{'pos':0,'eff':0.2,'k_eff':1.0}})
                # self.JC.set_eff('left_ankle',-3)
                # self.JC.set_eff('hip',-5.5)
                # self.JC.set_pos('right_ankle',0.2)
                # self.JC.send_command()
                rospy.sleep(0.1)
                self.CON.set_ref({'hip':{'pos':0,'eff':5.5,'k_eff':1.0},
                                  'left_ankle':{'pos':0,'eff':0,'k_eff':1.0},
                                  'right_ankle':{'pos':0,'eff':0.2,'k_eff':1.0}})
                # self.JC.set_eff('left_ankle',0)
                # self.JC.send_command()
                rospy.sleep(0.55)
                self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
                                  'left_ankle':{'pos':0,'eff':0,'k_eff':1.0},
                                  'right_ankle':{'pos':0,'eff':1,'k_eff':1.0}})
                # self.JC.set_eff('hip',0)
                # self.JC.set_eff('right_ankle',1)
                # self.JC.send_command()
                rospy.sleep(0.1)

        def TakeStep(self,which):
            if self.TimeElapsed<60:
                Aperture = abs(self.JointsState.position[0])
                self.SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
                # self.ToeOffEff = self.ToeOffEff0 + self.kAp*(self.DesAp - Aperture)
                
                # print Aperture
            else:
                Aperture = abs(self.JointsState.position[0])
                self.SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
                #self.ToeOffEff = self.ToeOffEff0 - 2*(self.DesAp - Aperture)
                #print self.ToeOffEff
                #print self.SwingEff

            if which == "right":
                # Toe off
                self.CON._joints['left_ankle'].set_params({'P': self.SupAnkGain_p, 'I': 0.0, 'D': self.SupAnkGain_d})
                self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
                                  'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'right_ankle':{'pos':0,'eff':self.ToeOffEff,'k_eff':1.0}})
                # self.JC.set_gains('left_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
                # self.JC.set_pos('left_ankle',self.SupAnkSetPoint)
                # self.JC.set_eff('right_ankle',self.ToeOffEff)
                # self.JC.set_eff('hip',0)
                # self.JC.send_command()
                rospy.sleep(self.ToeOffDur)
                # Raise feet and swing
                self.CON._joints['right_ankle'].set_params({'P': 35, 'I': 0.0, 'D': 3})
                self.CON.set_ref({'hip':{'pos':0,'eff':-self.SwingEff,'k_eff':1.0},
                                  'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'right_ankle':{'pos':-1.57,'eff':0,'k_eff':0.0}})
                # # self.JC.set_eff('right_ankle',-3)
                # self.JC.set_gains('right_ankle',30,15,0,set_default = False)
                # self.JC.set_pos('right_ankle',-1.57)
                # self.JC.set_eff('hip',self.SwingEff)
                # self.JC.send_command()
                rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
                # Lower feet
                self.CON.set_ref({'hip':{'pos':0,'eff':-self.SwingEff,'k_eff':1.0},
                                  'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'right_ankle':{'pos':self.DesAp,'eff':0,'k_eff':0.0}})
                # self.JC.set_eff('right_ankle',self.FootExtEff)
                # self.JC.send_command()
                rospy.sleep(self.FootExtDur)
                # End swing
                self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
                                  'left_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'right_ankle':{'pos':self.DesAp,'eff':0,'k_eff':1.0}})
                # self.JC.set_eff('hip',0)
                # self.JC.set_eff('right_ankle',0)
                # self.JC.send_command()
                rospy.sleep(self.FreeSwingDur)

            if which == "left":
                # Toe off
                self.CON._joints['right_ankle'].set_params({'P': self.SupAnkGain_p, 'I': 0.0, 'D': self.SupAnkGain_d})
                self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
                                  'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'left_ankle':{'pos':0,'eff':self.ToeOffEff,'k_eff':1.0}})
                # self.JC.set_gains('right_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
                # self.JC.set_pos('right_ankle',self.SupAnkSetPoint)
                # self.JC.set_eff('left_ankle',self.ToeOffEff)
                # self.JC.set_eff('hip',0)
                # self.JC.send_command()
                rospy.sleep(self.ToeOffDur)
                # Raise feet and swing
                self.CON._joints['left_ankle'].set_params({'P': 35, 'I': 0.0, 'D': 3})
                self.CON.set_ref({'hip':{'pos':0,'eff':self.SwingEff,'k_eff':1.0},
                                  'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'left_ankle':{'pos':-1.57,'eff':0,'k_eff':0.0}})
                # # self.JC.set_eff('right_ankle',-3)
                # self.JC.set_gains('left_ankle',30,15,0,set_default = False)
                # self.JC.set_pos('left_ankle',-1.57)
                # self.JC.set_eff('hip',self.SwingEff)
                # self.JC.send_command()
                rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
                # Lower feet
                self.CON.set_ref({'hip':{'pos':0,'eff':self.SwingEff,'k_eff':1.0},
                                  'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'left_ankle':{'pos':self.DesAp,'eff':0,'k_eff':0.0}})
                # self.JC.set_eff('left_ankle',self.FootExtEff)
                # self.JC.send_command()
                rospy.sleep(self.FootExtDur)
                # End swing
                self.CON.set_ref({'hip':{'pos':0,'eff':0,'k_eff':1.0},
                                  'right_ankle':{'pos':self.SupAnkSetPoint,'eff':0,'k_eff':0.0},
                                  'left_ankle':{'pos':self.DesAp,'eff':0,'k_eff':1.0}})
                # self.JC.set_eff('hip',0)
                # self.JC.set_eff('left_ankle',0)
                # self.JC.send_command()
                rospy.sleep(self.FreeSwingDur)


    eff_pos_node = EffPosNode()
    eff_pos_node.TimeOut = 300
    eff_pos_node.Go=0
    rospy.Subscriber("/gcb/joint_states", JointState, eff_pos_node.on_state_update)
    rospy.Subscriber('/ground_truth_odom',Odometry, eff_pos_node.Odom_cb)
    eff_pos_node.pid_param_update()
    # Finished initializing the controller

    # Start with right foot raised
    InitPose = zeros(3)
    InitPose[0] = -0.1
    InitPose[1] = 0.06
    InitPose[2] = -1.45

    eff_pos_node.ResetToPose(InitPose,2)

    # Save start time
    eff_pos_node.StartTime = rospy.Time.now().to_sec()

    # Take first step
    eff_pos_node.TakeFirstStep("right")
    StepsTaken = 1

    # Initialize aperture measurement
    ApertureMean = abs(eff_pos_node.JointsState.position[0])
    ApertureStd = 0

    # Rinse and repeat
    Leg = "left"
    eff_pos_node.Go=1
    while eff_pos_node.Go == 1:
        # Take another step
        eff_pos_node.TakeStep(Leg)
        StepsTaken += 1

        # ################################################################
        ThisAperture = abs(eff_pos_node.JointsState.position[0])
        ApertureMean += (ThisAperture-ApertureMean) / StepsTaken
        ApertureStd += ((ThisAperture-ApertureMean)**2 - ApertureStd) / StepsTaken

        if Leg == "right":
            Leg = "left"
        else:
            Leg = "right"

    print "Runtime: ",eff_pos_node.TimeElapsed," - Steps taken: ",StepsTaken
    print "Step size: ",2*sin(ApertureMean/2)," (+/-",2*sin(ApertureStd/2),")"

    # rospy.spin()


