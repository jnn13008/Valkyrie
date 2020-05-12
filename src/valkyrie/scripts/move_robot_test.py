#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from math import pi
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianPath:
    def __init__(self):
        rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        rospy.loginfo("Start roscpp_initialize")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("Start roscpp_initialize DONE")

        # Initialize the move group for the ur5_arm
        rospy.loginfo("Start cereate arm with MoveGroupCommander")
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        rospy.loginfo("Start cereate arm with MoveGroupCommander DONE")

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)


       	joint_goal = self.arm.get_current_joint_values()
        # printself.joint_goal
        joint_goal[0] = 0
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/4
        joint_goal[3] = -pi/2 - pi/4
        joint_goal[4] = -pi/2
        joint_goal[5] = pi/3
        #self.joint_goal[6] = 0
        print "joint_goal = %s" % str(joint_goal)
        self.arm.go(joint_goal, wait=True)

        # Get the current pose so we can add it as a waypoint
        self.start_pose = self.arm.get_current_pose(end_effector_link).pose
	# Mycode

        # Initialize the waypoints list
        # waypoints = []

        # # Set the first waypoint to be the starting pose
        # # Append the pose to the waypoints list
        # waypoints.append(start_pose)

        # wpose = deepcopy(start_pose)

        # # Set the next waypoint to the right 0.5 meters
        # wpose.position.x -= -1.1

        # waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
	    self.plan_x(0.1)
            (plan, fraction) = self.arm.compute_cartesian_path (self.waypoints, 0.01 , 0.0, True)
	    # rospy.loginfo("plan =\n{}".format(plan))
            # Increment the number of attempts
            attempts += 1

            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            ret = False
            attempts = 0
            while attempts < maxtries or ret:
                ret = self.arm.execute(plan)
                # rospy.loginfo("attempts={}, \tret={}".format(attempts, "Works" if ret else "Did not work"))
                attempts += 1
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

    def plan_x(self,x):
        self.waypoints = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        self.waypoints.append(self.start_pose)
        wpose = deepcopy(self.start_pose)
        # Set the next waypoint to the right 0.5 meters
        wpose.position.x += x
        wpose.position.z += -0.1

        self.waypoints.append(deepcopy(wpose))

    def cleanup(self):
       	joint_goal = self.arm.get_current_joint_values()
        # printself.joint_goal
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        #self.joint_goal[6] = 0
        self.arm.go(joint_goal, wait=True)
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        r=MoveItCartesianPath()
        r.cleanup()
    except KeyboardInterrupt:
        print "Shutting down MoveItCartesianPath node."
