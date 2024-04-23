import rospy
from mavros_msgs.msg import GlobalPositionTarget,State
from geometry_msgs.msg import PoseStamped

class PX4Controller:
    def __init__(self):
        rospy.init_node("airvoltex_offboard_node")
        # msg define
        self.arm_state = False
        self.current_state = None
        self.auto_control_flag=None
        self.local_position=None
        self.global_position=None
        # sub topic
        self.current_state_sub = rospy.Subscriber("/mavros/state",State,self.current_satate_callback)
        self.local_position_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.local_position_callback)
        self.global_position_sub = rospy.Subscriber("/mavros/global_position/pose",PoseStamped,self.global_position_callback)
        # pub topic
        

        # services

        #callback    
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

    








