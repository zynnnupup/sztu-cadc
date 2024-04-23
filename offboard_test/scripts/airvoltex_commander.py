#! /usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointReached,ActuatorControl,StatusText,Altitude
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import Float32, String
import time
import numpy as np


class Commander:

    def __init__(self):
        rospy.init_node("matilda_commander_node")

        # params
        self.direction=225 # in degrees
        self.drop_height = 20
        self.threshhold=10
        self.delta_T=0.05 # positive means drop befort than plan
        self.route=[]
        self.route_param=[10,120,50,60] # L0,L1,L2,L3
        self.left_into=False
        
        self.point_now=0
        self.direction=self.direction*np.pi/180
        self.altitude=None
        self.local_position = None
        self.local_velocity = None
        self.cur_target_position = None
        self.mission_reached=-1
        self.target_position_local=None
        self.search_finished = False
        self.target_finished = False
        self.attack_finished = False
        self.open_magazine = [0,0,0,1,0,0,0,0]
        self.close_magazine = [0,0,0,-1,0,0,0,0]

        self.local_position_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.local_position_callback)
        self.altitude_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.altitude_callback)
        self.local_velocity_sub = rospy.Subscriber("/mavros/local_position/velocity_local",TwistStamped,self.local_velocity_callback)
        self.mission_reached_sub = rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.mission_reached_callback)
        self.target_position_local_sub = rospy.Subscriber("/matilda_control/local_target",PoseStamped,self.target_position_local_callback)
        self.matilda_status_sub = rospy.Subscriber("/matilda_control/status",String,self.matilda_status_callback)

        self.set_target_position_pub = rospy.Publisher('/matilda_control/set_position/pose',PoseStamped,queue_size=10)
        self.mavros_status_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=5)
        self.matilda_status_pub = rospy.Publisher('/matilda_control/status',String,queue_size=10)
        self.actuator_pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=5)

        self.flight_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    
    def local_position_callback(self, msg):
        self.local_position = msg

    def altitude_callback(self,msg):
        self.altitude=msg

    def local_velocity_callback(self, msg):
        self.local_velocity = msg
    
    def mission_reached_callback(self, msg):
        self.mission_reached = msg.wp_seq

    def target_position_local_callback(self, msg):
        self.target_position_local=msg
        self.matilda_status_pub.publish(String('Targeted'))

    def matilda_status_callback(self,msg):
        if msg.data=='Attacked':
            self.attack_finished=True
        elif msg.data=='Searched':
            self.search_finished=True
        elif msg.data=='Targeted':
            self.target_finished=True

    def generate_pose(self, x=0, y=0, z=30):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z+self.altitude.relative-self.local_position.pose.position.z
        return pose
    
    def set_pose(self,pose):
        self.cur_target_position=pose
        self.set_target_position_pub.publish(pose)

    def reach_point(self, cur_p, target_p, threshold=10):
        distance=((cur_p.pose.position.x-target_p.pose.position.x)**2+(cur_p.pose.position.y-target_p.pose.position.y)**2)**0.5
        if distance<=threshold:
            return True
        else:
            return False

    def route_generator(self):
        target=PoseStamped()
        target.header.frame_id='map'
        target.pose=self.target_position_local.pose
        self.route=[]
        target.pose.position.z=self.drop_height
        self.route.append(target)
        point0=PoseStamped()
        point0.header.frame_id='map'
        point0.pose.position.x=target.pose.position.x+self.route_param[0]*np.sin(self.direction)
        point0.pose.position.y=target.pose.position.y+self.route_param[0]*np.cos(self.direction)
        point0.pose.position.z=target.pose.position.z
        self.route.append(point0)
        point2=PoseStamped()
        point2.header.frame_id='map'
        point2.pose.position.x=target.pose.position.x-self.route_param[1]*np.sin(self.direction)
        point2.pose.position.y=target.pose.position.y-self.route_param[1]*np.cos(self.direction)
        point2.pose.position.z=target.pose.position.z
        self.route.insert(0,point2)
        point3=PoseStamped()
        point3.header.frame_id='map'
        point3.pose.position.x=point2.pose.position.x-self.route_param[2]*np.sin(self.direction)
        point3.pose.position.y=point2.pose.position.y-self.route_param[2]*np.cos(self.direction)
        point3.pose.position.z=target.pose.position.z+5
        self.route.insert(0,point3)
        point4=PoseStamped()
        point4.header.frame_id='map'
        point5=PoseStamped()
        point5.header.frame_id='map'
        if self.left_into:
            point4.pose.position.x=point3.pose.position.x-self.route_param[3]*np.sin(self.direction+np.pi/6)
            point4.pose.position.y=point3.pose.position.y-self.route_param[3]*np.cos(self.direction+np.pi/6)
            point4.pose.position.z=point3.pose.position.z+4
            point5.pose.position.x=point3.pose.position.x-self.route_param[3]*np.sqrt(3)*np.sin(self.direction+np.pi/3)
            point5.pose.position.y=point3.pose.position.y-self.route_param[3]*np.sqrt(3)*np.cos(self.direction+np.pi/3)
            point5.pose.position.z=point3.pose.position.z+8
        else:
            point4.pose.position.x=point3.pose.position.x-self.route_param[3]*np.sin(self.direction-np.pi/6)
            point4.pose.position.y=point3.pose.position.y-self.route_param[3]*np.cos(self.direction-np.pi/6)
            point4.pose.position.z=point3.pose.position.z+4
            point5.pose.position.x=point3.pose.position.x-self.route_param[3]*np.sqrt(3)*np.sin(self.direction-np.pi/3)
            point5.pose.position.y=point3.pose.position.y-self.route_param[3]*np.sqrt(3)*np.cos(self.direction-np.pi/3)
            point5.pose.position.z=point3.pose.position.z+8
        self.route.insert(0,point4)
        self.route.insert(0,point5)

    def construct_actuator_control_target(self, group_mix,controls):
        target_actuator_control=ActuatorControl()
        target_actuator_control.header.stamp=rospy.Time.now()
        target_actuator_control.group_mix=group_mix
        target_actuator_control.controls=controls
        return target_actuator_control
    
    def drop(self):
        horizontal_vel=np.sqrt(self.local_velocity.twist.linear.x**2+self.local_velocity.twist.linear.y**2)
        flight_time=np.sqrt(2*self.altitude.relative/9.8)+self.delta_T
        if self.reach_point(self.local_position,self.target_position_local,horizontal_vel*flight_time):
            return True
        else:
            return False
        
    def cal_error(self):
        flight_time=np.sqrt(2*self.altitude.relative/9.8)
        return [-self.target_position_local.pose.position.x+(self.local_position.pose.position.x+flight_time*self.local_velocity.twist.linear.x),\
                -self.target_position_local.pose.position.y+(self.local_position.pose.position.y+flight_time*self.local_velocity.twist.linear.y)]

    def status_text(self, text, severity=6):
        send_status = StatusText()
        send_status.header.stamp = rospy.Time.now()
        send_status.severity = severity
        send_status.text = text
        return send_status        

    def start(self):
        mission_flag=0
        while not rospy.is_shutdown():
            if mission_flag==0:
                time.sleep(0.1)
                if self.target_finished:
                    mission_flag=1

            elif mission_flag==1:
                self.route_generator()
                self.set_pose(self.route[0])
                mission_flag=2

            elif mission_flag==2:
                for _ in range(2):
                    self.flight_mode_service(custom_mode='OFFBOARD')
                    time.sleep(0.1)
                mission_flag=3
                self.set_pose(self.route[self.point_now])


            elif mission_flag==3:
                time.sleep(0.1)
                if self.reach_point(self.local_position,self.cur_target_position,threshold=15):
                    self.point_now+=1
                    self.set_pose(self.route[self.point_now])
                if self.point_now==3:
                    mission_flag=4

            elif mission_flag==4:
                time.sleep(0.1)
                if self.reach_point(self.local_position,self.cur_target_position,threshold=self.threshhold):
                    self.point_now+=1
                    self.set_pose(self.route[self.point_now])
                    mission_flag=5

            elif mission_flag==5:
                if self.drop():
                    self.actuator_pub.publish(self.construct_actuator_control_target(2,self.open_magazine))
                    error=self.cal_error()
                    print(str(error))
                    self.mavros_status_pub.publish(self.status_text('Error'+str(error)))
                    self.matilda_status_pub.publish(String('Attacked'))
                    mission_flag=6

            elif mission_flag==6:
                time.sleep(1)


if __name__ == "__main__":
    con = Commander()
    con.start()
