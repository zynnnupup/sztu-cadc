import rospy
from mavros_msgs.msg import GlobalPositionTarget,State,ManualControl,RCIn,WaypointList,WaypointReached,Range,PositionTarget,GlobalPositionTarget,ActuatorControl,StatusText
from mavros_msgs.srv import CommandBool,SetMode,WaypointSetCurrent
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import roslaunch

class PX4Controller:
    def __init__(self):
        rospy.init_node("airvoltex_offboard_node")

        # msg define
        self.REAL_ENV = False

        self.arm_state = False
        self.current_state = None
        self.auto_control_flag = None
        self.local_position = None
        self.global_position = None
        self.manual_control = None
        self.rc_channels = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.mission_list = None
        self.mission_reached = -1
        self.current_target_position = None
        self.attack_finished = False
        self.search_finished = False

        # sub topic
        self.current_state_sub = rospy.Subscriber("/mavros/state",State,self.current_satate_callback)
        self.local_position_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.local_position_callback)
        self.global_position_sub = rospy.Subscriber("/mavros/global_position/pose",PoseStamped,self.global_position_callback)
        self.manual_control_sub = rospy.Subscriber("/mavros/manual_control/control",ManualControl,self.manual_control_callback)
        self.rc_in_sub = rospy.Subscriber("/mavros/rc/in",RCIn,self.rc_in_callback)
        self.distance_sensor_sub = rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, self.distance_sensor_callback)
        self.mission_list_sub = rospy.Subscriber("/mavros/mission/waypoints",WaypointList,self.mission_list_callback)
        self.mission_reached_sub = rospy.Subscriber("/mavros/mission/reached",WaypointReached,self.mission_reached_callback)

        self.set_target_position_sub = rospy.Subscriber("/airvoltex_control/set_position/pose",PoseStamped,self.set_target_position_callback)
        self.airvoltex_status_sub = rospy.Subscriber("/airvoltex_control/status",String,self.airvoltex_status_callback)
        
        # pub topic
        self.local_target_pub = rospy.Publisher("/mavros/setpoint_raw/local",PositionTarget,queue_size=5)
        self.global_target_pub = rospy.Publisher("/mavros/setpoint_raw/global",GlobalPositionTarget,queue_size=5)
        self.actuator_pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=5)
        self.mavros_status_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=5)
        
        # services
        self.arm_service = rospy.ServiceProxy("mavros/cmd/arming",CommandBool)
        self.flight_mode_service = rospy.ServiceProxy("/mavros/set_mode",SetMode)
        self.set_current_waypoint_service = rospy.ServiceProxy("/mavros/mission/set_current",WaypointSetCurrent)


        print("PX4 Controller Initialized!")
        self.mavros_status_pub.publish(self.status_text("PX4 Controller Initialized!"))

    # callback    
    def current_satate_callback(self,msg):
        self.current_state = msg.mode
        self.arm_state = msg.armed
        if self.current_state == "MANUAL" or self.current_state == "ACRO" or self.current_state == "ALTCTL" or self.current_state == "POSCTL" or self.current_state == "STABILIZED" or self.current_state == "RATTITUDE":
            self.auto_control_flag=False
        else:
            self.auto_control_flag=True
        if rospy.is_shutdown():
            exit()

    def local_position_callback(self,msg):
        self.local_position = msg
    
    def global_position_callback(self,msg):
        self.global_position = msg

    def manual_control_callback(self,msg):
        self.manual_control = msg

    def rc_in_callback(self,msg):
        self.rc_channels = msg.channels

    def distance_sensor_callback(self,msg):
        self.lidar_distance = msg.range

    def mission_list_callback(self,msg):
        self.mission_list = msg

    def mission_reached_callback(self,msg):
        self.mission_reached = msg.wp_seq

    def set_target_position_callback(self,msg):
        if self.msg.header.frame_id == 'map':
            self.current_target_position = self.construt_local_target(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
        else:
            print('Unknown Frame:',msg.header.frame_id)
    
    def airvoltex_status_callback(self,msg):
        if msg.data == "Attacked":
            self.attack_finished = True
        elif msg.data == "Searched":
            self.search_finished = True
        elif msg.data == "Targeted":
            self.target_finished = True


    # other methods

    def construt_local_target(self,x,y,z):
        target_local_position = PositionTarget()
        target_local_position.header.stamp = rospy.Time.now()
        target_local_position.coordinate_frame = 1  # MAV_FRAME_LOCAL_NED
        target_local_position.position.x = x
        target_local_position.position.y = y
        target_local_position.position.z = z


        target_local_position.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW \
            + PositionTarget.IGNORE_YAW_RATE + PositionTarget.FORCE
        
        return target_local_position

    def start_a_launchfile(self,path):
        uuid = roslaunch.rlutil.grt_or_generate_uuid(None,False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[path])
        launch.start()
        return  launch
    
    def construct_actuator_control_target(self,group_mix,controls):
        target_actuator_control = ActuatorControl()
        target_actuator_control.header.stamp = rospy.Time.now()
        target_actuator_control.group_mix = group_mix
        target_actuator_control.controls = controls
        return target_actuator_control
    
    def status_text(self, text, severity=6):
        send_status = StatusText()
        send_status.header.stamp = rospy.Time.now()
        send_status.severity = severity
        send_status.text = text
        return send_status
    # start

    def start(self):
        mission_flag = 0
        while not rospy.is_shutdown():
            rospy.loginfo("mission_flag:%d",mission_flag)
            if mission_flag == 0:
                if self.local_position == None:
                    time.sleep(0.1)
                else:
                    mission_flag = 1
                    self.current_target_position = self.construt_local_target(self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)
                    
            elif mission_flag == 1:
                for _ in range(10):
                    self.local_target_pub.publish(self.current_target_position)
                    time.sleep(0.1)
                mission_flag = 2
            
            elif mission_flag == 2:
                self.local_target_pub.publish(self.current_target_position)
                
                #real flight
                if self.REAL_ENV:
                    rospy.loginfo("real mode!")
                    if self.rc_channels[4] > 1750:
                        time.sleep(0.1)
                    else:
                        mission_flag = 3
                
                #simulation
                else:
                    rospy.loginfo("simulation mode!")
                    mission_flag = 3
            
            elif mission_flag == 3:
                #take off
                self.local_target_pub.publish(self.current_target_position)
                if(not self.arm_state) or (not self.current_state == 'AUTO.MISSION'):
                    time.sleep(0.1)

                    if self.REAL_ENV == False:
                        self.arm_service(True)
                        self.flight_mode_service(custom_mode = 'AUTO.MISSION')
                else:
                    mission_flag = 4
            
            elif mission_flag == 4:
                if self.arm_state and self.current_state == 'AUTO.MISSION':
                    rospy.loginfo("MISSION!")
                    self.mavros_status_pub.publish(self.status_text("AUTO TAKEOFF!"))
                    mission_flag = 5
                else:
                    mission_flag = 3
            
            elif mission_flag == 5:
                if(not self.arm_state) or (not self.current_state == 'AUTO.MISSION'):
                    time.sleep(0.1)
                    self.local_target_pub.publish(self.current_target_position)
                else:
                    mission_flag = 6
                    self.mavros_status_pub.publish(self.status_text("MISSION!"))
            
            #wait for takeoff
            elif mission_flag == 6:
                self.set_current_waypoint_service(wp_seq = 0)
                if self.mission_reached < 0:
                    time.sleep(0.1)
                    self.local_target_pub.publish(self.current_target_position)
                else:
                    mission_flag = 7

            #search and attack
            elif mission_flag == 7:
                #todo launch file 
                searching = self.start_a_launchfile("")
                self.local_target_pub.publish(self.current_target_position)
                mission_flag = 8
            
            #searching 
            elif mission_flag == 8:
                if not self.search_finished:
                    time.sleep(0.1)
                    self.local_target_pub.publish(self.current_target_position)
                else:
                    searching.shutdown()
                    mission_flag = 9
            
            elif mission_flag == 9:
                for _ in range(2):
                    self.flight_mode_service( custiom_mode = 'AUTO.LOITER')
                    time.sleep(0.1)
                else:
                    mission_flag = 10
            
            elif mission_flag == 10:
                if not self.target_finished:
                    self.local_target_pub.publish(self.current_target_position)
                    time.sleep(0.1)
                else:
                    mission_flag = 11

            elif mission_flag == 11:
                for _ in range(2):
                    self.local_target_pub.publish(self.current_target_position)
                    self.flight_mode_service(custom_mode = 'OFFBOARD')
                    time.sleep(0.1)
                mission_flag = 12
            
            elif mission_flag == 12:
                if not self.attack_finished:
                    time.sleep(0.1)
                    self.local_target_pub.publish(self.current_target_position)
                else:
                    mission_flag = 98
                    if self.REAL_FLIGHT:
                        self.actuator_pub.publish(self.construct_actuator_control_target(2,self.open_magazine))
            
            #land
            elif mission_flag == 98:
                for _ in range(2):
                    self.set_current_waypoint_service(wp_seq = 18)
                    self.flight_mode_service(custom_mode = 'AUTO.MISSION')
                    time.sleep(0.1)
                mission_flag = 99

            elif mission_flag == 99:
                time.sleep(0.1)
                self.local_target_pub.publish(self.current_target_position)

            else:
                exit()
    
if __name__ == '__main__':
    con = PX4Controller()
    con.start()
            
                

            





             








