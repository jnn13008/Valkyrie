#!/usr/bin/env python
import rospy
from valkyrie.srv import *
import valkyrie.srv as valkyrie

#serv = "valkyrie/{}state"
serv = "valkyrie/{}state"

def setRobotState(state,comment):
    s = serv.format("set")
    rospy.loginfo("wait for " + s)
    rospy.wait_for_service(s)
    rospy.loginfo("[OK] " + s)
    setState = rospy.ServiceProxy(s, valkyrie.setState)
    print setState
    return setState(state, comment)



def getRobotState():
    s = serv.format("get")
    rospy.loginfo("wait for " + s)
    rospy.wait_for_service(s)
    rospy.loginfo("[OK] " + s)
    getState = rospy.ServiceProxy(s,valkyrie.getState)
    return getState()


def main():
    rospy.init_node("test_state")
    rospy.loginfo("Start")
    rospy.loginfo("robot State = \n" + str(getRobotState()))
    rospy.loginfo("Writing new state to robot")
    rospy.loginfo("returned " + str(setRobotState(2,"World")))
    rospy.loginfo("robot State = \n" + str(getRobotState()))


if __name__ == '__main__':
    main()
