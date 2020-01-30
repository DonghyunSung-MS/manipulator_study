#! /usr/bin/env python

import rospy
import sys
import time

def settingParameters():
    rospy.init_node("Terminal_UI",anonymous=False);
    limits = ["-166 ~ 166 deg",
                "-101 ~ 101 deg",
                "-166 ~ 166 deg",
                "-176 ~ -04 deg",
                "-166 ~ 166 deg",
                "-001 ~ 215 deg",
                "-166 ~ 166 deg"]
    while not rospy.is_shutdown():
        while(True):
            try:
                tmp = input("Target Execute time : ")
                break
            except:
                rospy.loginfo("Please Enter Number")
        for i in range(7):
                while(True):
                    try:
                        temp = input("Target Franka_joint"+str(i+1)+"(limit "+limits[i]+") : ")
                        break
                    except:
                        rospy.loginfo("Please Enter Number")
        rospy.set_param("vrepRosPosControl2/exec_time",tmp)
        for i in range(7):
            rospy.set_param("vrepRosPosControl2/j"+str(i+1),temp)

        rospy.loginfo("Wait For Manipulator moved")
        time.sleep(5)

if __name__=='__main__':
    settingParameters()
