#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *

class Controller:
    def __init__(self):
        self.sp                  = PositionTarget()
        self.sp.type_mask        = int('010111111000', 2)
        self.sp.coordinate_frame = 1
        self.sp.position.z       = 0.7 # Altitude setpoint
        self.localpos            = Point(0.0, 0.0, 0.0)
        self.dronepos            = Point(0.0, 0.0, 0.0)
        self.framepos            = Point(0.0, 0.0, 0.0)
        self.xoffset = 1.0
        self.yoffset = 1.0

    def localPosCallback(self, msg):
        self.localpos.x = msg.pose.position.x
        self.localpos.y = msg.pose.position.y
        self.localpos.z = msg.pose.position.z

    def framePosCallback(self, msg):
        self.framepos = msg.pose.position

    def dronePosCallback(self, msg):
        self.dronepos = msg.pose.position        

    def update(self):
        self.sp.position.x = self.framepos.x
        self.sp.position.y = self.framepos.y - self.yoffset
    
# Main function
def main():

    rospy.init_node('moving_frame', anonymous = True)
    cnt  = Controller()
    rate = rospy.Rate(20.0)

    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.localPosCallback)
    rospy.Subscriber('vrpn_client_node/frame_origin/pose', PoseStamped, cnt.framePosCallback)
    rospy.Subscriber('vrpn_client_node/drone/pose', PoseStamped, cnt.dronePosCallback)

    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size = 1)

    # Continuous loop publisher
    while not rospy.is_shutdown():
        cnt.update()
        sp_pub.publish(cnt.sp)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
