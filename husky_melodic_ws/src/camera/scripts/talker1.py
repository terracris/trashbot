#!/usr/bin/env python2
#coding=utf-8
#import roslib;roslib.load_manifest('beginner_tutorials')
import rospy
from beginner_tutorials.msg import Num

def talker():
    pub = rospy.Publisher('chatter', Num, queue_size = 10)
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mynum = int(rospy.get_time())
        msg = Num()
        msg.num = mynum
        rospy.loginfo(mynum)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
