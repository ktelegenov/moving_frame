#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Bool
import time
import math 

# class fcuModes:
#     def __init__(self):
#         pass

#     def setAutoLandMode(self):
#         rospy.wait_for_service('mavros/set_mode')
#         try:
#             flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
#             flightModeService(custom_mode='AUTO.LAND')
#         except rospy.ServiceException, e:
#               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    def __init__(self):
        self.state               = State()
        self.sp                  = PositionTarget()
        self.sp.type_mask        = int('010111111000', 2)
        self.sp.coordinate_frame = 1
        self.sp.position.z       = 0.7 # Altitude setpoint for height
        
        self.localpos            = Point(0.0, 0.0, 0.0)
        self.dronepos            = Point(0.0, 0.0, 0.0)
        self.framepos            = Point(0.0, 0.0, 0.0)

        self.xoffset = 1.0
        self.yoffset = 1.0

        # self.modes               = fcuModes()
        # self.next_point          = 0
        # self.first_time          = 0

        # States
        # self.takeoff = 0
        # self.p1		 = 0
        # self.p2      = 0
        # self.p3		 = 0
        # self.p4	     = 0
        # self.hover	 = 0
        # self.land	 = 0

        # X/Y coordinates of points of interest
        # self.p1_X = -2.0
        # self.p1_Y =  2.5
        # self.p2_X =  2.0
        # self.p2_Y =  2.5
        # self.p3_X =  2.0
        # self.p3_Y = -2.5
        # self.p4_X = -2.0
        # self.p4_Y =  -2.5
        
    # def resetStates(self):
    # 	self.takeoff = 0
    #     self.p1		 = 0
    #     self.p2      = 0
    #     self.p3		 = 0
    #     self.p4	     = 0
    #     self.hover 	 = 0
    #     self.land    = 0

    def stateCb(self, msg):
        self.state = msg

    def localPosCallback(self, msg):
        self.localpos.x = msg.pose.position.x
        self.localpos.y = msg.pose.position.y
        self.localpos.z = msg.pose.position.z

    def framePosCallback(self, msg):
        self.framepos = msg

    def dronePosCallback(self, msg):
        self.dronepos = msg

    def somefunction(self):
        self.sp.position.x = cnt.framepos.x + xoffset
        self.sp.position.y = cnt.framepos.y + yoffset
    

# Main function
def main():

    rospy.init_node('moving_frame', anonymous = True)
    cnt  = Controller()
    rate = rospy.Rate(20.0)

    # rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.localPosCallback)
    rospy.Subscriber('vrpn_client_node/frame/pose', PoseStamped, cnt.framePosCallback)
    rospy.Subscriber('vrpn_client_node/drone/pose', PoseStamped, cnt.dronePosCallback)

    pos_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size = 1)


    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size = 1)

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k < 10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # Prepare for a hover at current point
    cnt.sp.position.x = cnt.local_pos.x
    cnt.sp.position.y = cnt.local_pos.y
    cnt.sp.position.z = cnt.alt_sp

    # Reset state machine
    cnt.resetStates()

    # Start with takeoff state
    cnt.takeoff = 1
    cnt.first_time = 1

    while not rospy.is_shutdown():
        if cnt.takeoff:
            rospy.loginfo("Taking off")
            if abs(cnt.local_pos.z - cnt.alt_sp) < 0.1:
                cnt.resetStates()
                cnt.p1 = 1

        if cnt.p1:
            rospy.loginfo("Heading to P1")
            
            cnt.sp.position.x = cnt.p1_X
            cnt.sp.position.y = cnt.p1_Y

            if math.sqrt((abs(cnt.local_pos.x - cnt.p1_X))**2 + (abs(cnt.local_pos.y - cnt.p1_Y))**2) < 0.1:
                if cnt.first_time == 1:
                    cnt.next_point = 2
                    cnt.first_time = 0
                else:
                    cnt.next_point = 5
                cnt.resetStates()
                cnt.hover  = 1
                cnt.hovering_time = time.time()

        if cnt.p2:
            rospy.loginfo("Heading to P2")
            
            cnt.sp.position.x = cnt.p2_X
            cnt.sp.position.y = cnt.p2_Y
            if math.sqrt((abs(cnt.local_pos.x - cnt.p2_X))**2 + (abs(cnt.local_pos.y - cnt.p2_Y))**2) < 0.1:
                cnt.next_point = 3
                cnt.resetStates()
                cnt.hover = 1
                cnt.hovering_time = time.time()

        if cnt.p3:
            rospy.loginfo("Heading to P3")
            
            cnt.sp.position.x = cnt.p3_X
            cnt.sp.position.y = cnt.p3_Y
            if math.sqrt((abs(cnt.local_pos.x - cnt.p3_X))**2 + (abs(cnt.local_pos.y - cnt.p3_Y))**2) < 0.1:
                cnt.next_point = 4
                cnt.resetStates()
                cnt.hover = 1
                cnt.hovering_time = time.time()

        if cnt.p4:
            rospy.loginfo("Heading to P4")
            
            cnt.sp.position.x = cnt.p4_X
            cnt.sp.position.y = cnt.p4_Y
            if math.sqrt((abs(cnt.local_pos.x - cnt.p4_X))**2 + (abs(cnt.local_pos.y - cnt.p4_Y))**2) < 0.1:
                cnt.next_point = 1
                cnt.resetStates()
                cnt.hover = 1
                cnt.hovering_time = time.time()

        if cnt.hover:
            rospy.loginfo("Hovering")
            cnt.sp.position.x = cnt.local_pos.x
            cnt.sp.position.y = cnt.local_pos.y

            if time.time() - cnt.hovering_time > 7:
                if cnt.next_point == 5:                    
                    cnt.resetStates()
                    cnt.land = 1

                if cnt.next_point == 1:
                    cnt.resetStates()
                    cnt.p1 = 1

                if cnt.next_point == 2:
                    cnt.resetStates()
                    cnt.p2 = 1

                if cnt.next_point == 3:
                    cnt.resetStates()
                    cnt.p3 = 1

                if cnt.next_point == 4:
                    cnt.resetStates()
                    cnt.p4 = 1

        if cnt.land:
            rospy.loginfo("Landing")
            cnt.modes.setAutoLandMode()

        sp_pub.publish(cnt.sp)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass