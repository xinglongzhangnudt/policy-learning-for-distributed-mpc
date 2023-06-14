#!/usr/bin/python
# -*- coding: UTF-8 -*-
### This code is about the distributed formation control with a consensus protocol
### For more details, please see the paper on https://arxiv.org/abs/2005.01125

from numpy.core.records import array
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TwistStamped
from std_msgs.msg import String
import sys
import numpy
import math
if sys.argv[3] == '6':
    from formation_dict import formation_dict_6 as formation_dict
elif sys.argv[3] == '9':
    from formation_dict import formation_dict_9 as formation_dict
elif sys.argv[3] == '18':
    from formation_dict import formation_dict_18 as formation_dict
else:
    print("Only 6, 9 and 18 UAVs are supported.")

class Follower:
    def __init__(self, uav_type, uav_id, uav_num, control_type):
        self.uav_type = uav_type
        self.id = uav_id
        self.uav_num = uav_num
        self.control_type = control_type
        self.f = 30
        self.formation_pattern = None
        self.communication_topology = None
        self.changed_id = numpy.arange(0, self.uav_num-1)
        self.local_leader_ids = []  
        self.Kp = 1.0
        self.Kp_avoid = 1.0
        self.weight = 1.0
        self.coefficient_y=0.7
        self.pose = [PoseStamped() for i in range(self.uav_num)]
        rospy.init_node('follower' + str(self.id ))
        self.pose_sub = [[] for i in range(self.uav_num)]
        self.formation_pattern = formation_dict["origin"]
        self.leader_cmd_sub = rospy.Subscriber("/xtdrone/leader/cmd",String, self.cmd_callback, queue_size=1)
        self.communication_topology=[[0,0,0,0,0,0],[1,0,0,0,0,0],[1,1,0,0,0,0],[1,0,1,0,0,0],[1,0,0,1,0,0],[1,0,0,0,1,0]]
        self.communication_topology=numpy.array(self.communication_topology)
        self.local_leader_ids = numpy.argwhere(self.communication_topology[self.id, :] == 1)
        self.local_leader_ids = self.local_leader_ids.reshape(self.local_leader_ids.shape[0])
        self.weight = self.Kp / len(self.local_leader_ids)
        self.cmd_pub = rospy.Publisher('/xtdrone/' + self.uav_type + '_' + str(self.id) + '/cmd', String, queue_size=1)

        ## self define variables
        self.StateBound1=[2,2,2,5,5]
        self.StateBound2=[2,2,2,5,5]
        self.W2=[20.1081875005841,-0.284196888476667,3.29068912613634,34.7618315371199,-4.25524095015186,
                 0.902496871148685,0.647431516025424,-2.54679133203095,3.82629593065434,-0.591938577639847,
                 14.8400518164609,9.32408392947442,24.3655553253728,20.0067456157513,73.9721129078117,
                 7.57012129353939,-2.45253533882473,-2.10324370333377,-9.54202643593251,2.28038516439110]
        self.leader_velocity = Twist()
        self.leader_vel_sub=rospy.Subscriber("/xtdrone/leader/cmd_vel_flu", Twist, self.leader_velocity_callback,queue_size=1)
        self.e1 = [[] for i in range(self.uav_num)]
        self.E2qBuf=[0,0,0,0,0]
        self.e2 =[0,0,0,0,0]
        self.C1Input=[0,0,0,0,0]
        self.C2Input=[0,0,0,0,0]
        self.tanhBuf2=[0,0,0,0,0,0,0,0,0,0]
        self.u=[0,0]

        for i in range(self.uav_num):
            self.pose_sub[i] = rospy.Subscriber(
            self.uav_type + '_' + str(i) + "/mavros/local_position/pose", PoseStamped,
            self.pose_callback, i, queue_size=1)
        if self.control_type == "vel":
            self.vel_max = 300
            self.cmd_vel_enu = Twist()
            self.avoid_vel = Vector3()
            self.avoid_vel_sub = rospy.Subscriber("/xtdrone/" + self.uav_type + '_' + str(self.id) + "/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
            self.vel_enu_pub = rospy.Publisher('/xtdrone/' + self.uav_type + '_' + str(self.id) + '/cmd_vel_enu', Twist, queue_size=1)
            self.velocity = [TwistStamped() for i in range(self.uav_num)]
            self.velocity_sub = [[] for i in range(self.uav_num)]
            for i in range(self.uav_num):
                self.velocity_sub[i] = rospy.Subscriber(
                self.uav_type + '_' + str(i) + "/mavros/local_position/velocity_local", TwistStamped,
                self.velocity_callback, i, queue_size=1)

    # self-define function
    def cmd_callback(self, msg):
        self.cmd = msg.data
        if self.cmd in ['origin', 'doubleorigin','T', 'diamond', 'triangle']:
            self.formation_pattern=formation_dict[self.cmd]
            self.formation_pattern[2,:]=0
            print(self.formation_pattern)
            self.coefficient_y=0.3
        if self.cmd in ['f','h']:
            self.coefficient_y=0.1

    def leader_velocity_callback(self,msg):
        self.leader_velocity = msg

    def quaternion_to_euler(self,msg):
        roll=math.atan2(2*(msg.w*msg.x+msg.y*msg.z),1-2*(msg.x*msg.x+msg.y*msg.y))
        pitch=math.asin(2*(msg.w*msg.y-msg.x*msg.z))
        yaw=math.atan2(2*(msg.w*msg.z+msg.x*msg.y),1-2*(msg.z*msg.z+msg.y*msg.y))
        return roll,pitch,yaw
    
    def pose_callback(self, msg, id):
        self.pose[id] = msg
    
    def velocity_callback(self, msg, id):
        self.velocity[id] = msg
    
    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def avoid_accel_callback(self, msg):
        self.avoid_accel = msg

  
    def controller(self):
         if (not self.communication_topology is None and not self.formation_pattern is None):
                input = Vector3(0, 0, 0)
                self.E2qBuf=[0,0,0,0,0]
                e1=[0,0,0,0,0]
                self.u=[0,0]
                if (self.control_type == "vel"):
                    for local_leader_id in self.local_leader_ids:
                        if local_leader_id == 0: # The local leader is the global leader
                            input.x += self.pose[local_leader_id].pose.position.x - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.id - 1])
                            input.y += self.pose[local_leader_id].pose.position.y - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.id - 1])
                            input.z += self.pose[local_leader_id].pose.position.z - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.id - 1])
                        else:
                            input.x += (self.pose[local_leader_id].pose.position.x - self.formation_pattern[0, local_leader_id - 1]) - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.id - 1])
                            input.y += (self.pose[local_leader_id].pose.position.y - self.formation_pattern[1, local_leader_id - 1]) - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.id - 1])
                            input.z += (self.pose[local_leader_id].pose.position.z - self.formation_pattern[2, local_leader_id - 1]) - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.id - 1])
                    self.cmd_vel_enu.linear.x = self.weight * input.x*4 + self.Kp_avoid * self.avoid_vel.x
                    self.cmd_vel_enu.linear.y = self.weight * input.y*4 + self.Kp_avoid * self.avoid_vel.y
                    self.cmd_vel_enu.linear.z = self.weight * input.z*4 + self.Kp_avoid * self.avoid_vel.z 
                    cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5
                    if (cmd_vel_magnitude > 3**0.5 * self.vel_max):
                        self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                        self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                        self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max

                    theta = 0 #follower
                    theta_self=0#self
                    theta_leader=0#leader
                    for local_leader_id in self.local_leader_ids:
                        local_leader_id=int(local_leader_id)
                        roll,pitch,theta_self= self.quaternion_to_euler(self.pose[self.id].pose.orientation)
                        if(local_leader_id==0): #leader
                            roll,pitch,theta_leader=self.quaternion_to_euler(self.pose[local_leader_id].pose.orientation)
                            self.E2qBuf[0]+=self.pose[local_leader_id].pose.position.x - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.id - 1])
                            self.E2qBuf[1]+=self.pose[local_leader_id].pose.position.y - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.id - 1])
                            input.z += self.pose[local_leader_id].pose.position.z - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.id - 1])
                    
                            self.E2qBuf[2]+=(theta_leader-theta_self)
                            if(self.E2qBuf[2]>2*math.pi-1):
                                self.E2qBuf[2]=self.E2qBuf[2]-2*math.pi
                            if(self.E2qBuf[2]<-2*math.pi+1):
                                self.E2qBuf[2]=self.E2qBuf[2]+2*math.pi
                            self.E2qBuf[3]+=(self.velocity[local_leader_id].twist.linear.x**2 + self.velocity[local_leader_id].twist.linear.y**2 )**0.5-(self.velocity[self.id].twist.linear.x**2 + self.velocity[self.id].twist.linear.y**2 )**0.5
                            self.E2qBuf[4]+=self.velocity[local_leader_id].twist.angular.z-self.velocity[self.id].twist.angular.z

                        elif(local_leader_id>0):
                            roll,pitch,theta = self.quaternion_to_euler(self.pose[local_leader_id].pose.orientation)
                            self.E2qBuf[0]+=self.pose[local_leader_id].pose.position.x-self.formation_pattern[0,local_leader_id-1]-(self.pose[self.id].pose.position.x-self.formation_pattern[0,self.id-1])
                            self.E2qBuf[1]+=self.pose[local_leader_id].pose.position.y-self.formation_pattern[1,local_leader_id-1]-(self.pose[self.id].pose.position.y-self.formation_pattern[1,self.id-1])
                            input.z += self.pose[local_leader_id].pose.position.z - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.id - 1])
                            self.E2qBuf[2]+=(theta-theta_self)
                            if(self.E2qBuf[2]>2*math.pi-1):
                                self.E2qBuf[2]=self.E2qBuf[2]-2*math.pi
                            if(self.E2qBuf[2]<-2*math.pi+1):
                                self.E2qBuf[2]=self.E2qBuf[2]+2*math.pi
                            self.E2qBuf[3]+=(self.velocity[local_leader_id].twist.linear.x**2 + self.velocity[local_leader_id].twist.linear.y**2 )**0.5-(self.velocity[self.id].twist.linear.x**2 + self.velocity[self.id].twist.linear.y**2 )**0.5
                            self.E2qBuf[4]+=self.velocity[local_leader_id].twist.angular.z-self.velocity[self.id].twist.angular.z
                    self.e2[0]=math.cos(theta_self)*self.E2qBuf[0]+math.sin(theta_self)*self.E2qBuf[1]
                    self.e2[1]=(-math.sin(theta_self)*self.E2qBuf[0]+math.cos(theta_self)*self.E2qBuf[1])*self.coefficient_y
                    self.e2[2]=self.E2qBuf[2]
                    self.e2[3]=self.E2qBuf[3]
                    self.e2[4]=self.E2qBuf[4]
                    list_4 = ' '.join(str(i) for i in self.e2)
                    list_5=(self.velocity[self.id].twist.linear.x**2 + self.velocity[self.id].twist.linear.y**2 )**0.5
                    with open('test_iris_'+str(self.id)+'.txt','a') as f:
                        f.write(str(list_4)+"\t")
                        f.write(str(list_5)+"\n")
                    with open('test_position_'+str(self.id)+'.txt','a') as g:
                        g.write(str(self.pose[self.id].pose.position.x)+"\t")
                        g.write(str(self.pose[self.id].pose.position.y)+"\t")
                        g.write(str(self.pose[self.id].pose.position.z)+"\t")
                        g.write(str(theta)+"\t")
                        g.write(str(self.velocity[self.id].twist.angular.z)+"\t")
                        g.write(str(theta_leader)+"\n")
                    self.vel_enu_pub.publish(self.cmd_vel_enu)
                    
    def loop(self):
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            self.controller()
            rate.sleep()  

if __name__ == '__main__':
    follower = Follower(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])
    follower.loop()