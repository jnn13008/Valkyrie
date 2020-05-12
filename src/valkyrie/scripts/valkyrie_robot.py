#!/usr/bin/env python
import sys
import copy
import rospy
rospy.loginfo("import state")
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import numpy as np
import valkyrie.srv as valkyrie
# import numpy as np

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from state import state

class objectview(object):
    def __init__(self, d):
        self.__dict__ = d

class robot(state):
    """ This class decribes how the robot works """
    # Class wariables
    eff_step = 100
    jump_th = 0.0
    _waypoints = list()
    _robot_plan = None
    _robot_fraction = None
    _robot_state = objectview({'home':0,'left':1,'right':2,'grasp':3})
    _wpscale = 1.0

    def __init__(self, port=None):
        # super(robot, self).__init__()
        state.__init__(self)
        rospy.loginfo("__init__  init_node")
        rospy.init_node("valkyrie_robot")
        rospy.loginfo("__init__  init_node done")

        # Init robot commander
        rospy.loginfo("__init__  moveit_commander init")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("__init__  moveit_commadeer init done")



        # Inscance the robot commander
        rospy.loginfo("__init__ moveit commander robot commander")
        self._robot =        moveit_commander.RobotCommander()
        rospy.loginfo("__init__ moveit commander robot commander done")

        self._robot_group_names =   self._robot.get_group_names()
        rospy.loginfo("_robot_group_names done")
        self._robot_name =          self._robot_group_names[1]
        rospy.loginfo("_robot_name= {}".format(self._robot_name))
        rospy.loginfo("Using {} to start moveit".format(self._robot_name))


        # Instance the planing sene inteface
        self._robot_scene =         moveit_commander.PlanningSceneInterface()

        rospy.loginfo("Scene created")
        try:
            self.arm = moveit_commander.MoveGroupCommander(self._robot_name)
            # self.arm = moveit_commander.MoveGroupCommander("arm")
            #self.arm.set_planner_id("PRMkConfigDefault")
            self.arm.set_planner_id("TRRT")
            self.arm.set_goal_tolerance(0.01)
            self.arm.set_goal_orientation_tolerance(0.1)
        except RuntimeError as e:
            rospy.logerr("You forgot to launch the ./ur5_simulated.sh")
            print("start ./ur5_simulated.sh")
            raise e
        # rospy.loginfo("[planer] {}".format(self.arm.get_planner_id() ))
        self.arm.clear_pose_targets()
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self._planning_frame = self.arm.get_planning_frame()
        rospy.loginfo("Planning frame: %s" % self._planning_frame)
        # We can also print the name of the end-effector link for this group:
        self._robot_eef_link = self.arm.get_end_effector_link()
        rospy.loginfo("End effector link: %s" % self._robot_eef_link)
        # We can get a list of all the groups in theself.robot:
        self.robot_go_home()
        pass

    def robot_go_home(self):
        joint_goal = self.arm.get_current_joint_values()
        print "joint_goal = %s" % str(joint_goal)
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
        self.arm.stop()

    def robot_go_left(self):
        joint_goal = self.arm.get_current_joint_values()
        print "joint_goal = %s" % str(joint_goal)
        # printself.joint_goal
        joint_goal[0] = -pi/4
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/4
        joint_goal[3] = -pi/2
        joint_goal[3] = -pi/2 - pi/4
        joint_goal[4] = -pi/2 + pi/4
        joint_goal[5] = pi/3
        #self.joint_goal[6] = 0
        print "joint_goal = %s" % str(joint_goal)
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()

    def robot_go_right(self):
        joint_goal = self.arm.get_current_joint_values()
        print "joint_goal = %s" % str(joint_goal)
        # printself.joint_goal
        joint_goal[0] = pi/4
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/4
        joint_goal[3] = -pi/2 - pi/4
        joint_goal[4] = -pi/2 - pi/4
        joint_goal[5] = pi/3
        #self.joint_goal[6] = 0
        print "joint_goal = %s" % str(joint_goal)
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()

    def _plan_path(self):
        """ Plans the path dependent on the robot state """
        last_state = self._state_log[-2] if len(self._state_log) > 1 else 0
        new_state = self._state
        w = self._wpscale
        dm = 1/10
        cm = 1/100
        mm = 1/1000
        # Check if the state have changed.
        rospy.loginfo("[_plan_path] Last_state={}, new_state={}".format(last_state,new_state))
        # if not new_state == last_state:
        if True:
            rospy.loginfo("run")
            # wpose contains che current pose.
            wpose = self.arm.get_current_pose().pose
            # new_state == self._robot_state.home:
            # if  new_state == self._robot_state.home:
            #     rospy.loginfo("[_plat_path] go_home")
            #     self.robot_go_home() # You are drunk
            #     # Re write the wpose because the robot pose changed
            #     wpose = self.arm.get_current_pose().pose
            # # elif new_state == self._robot_state.left and last_state == self._robot_state.home:
            # elif new_state == self._robot_state.left :
            rospy.loginfo("[_plat_path] go left")
            # self._waypoints.append(copy.deepcopy(wpose))
            # wpose.position.x += w * 2 * cm # and sideways (y)
            # wpose.position.y += w * 3 * cm # and sideways (y)
            wpose.position.z += w * np.random.random_sample()*50 - 25 * cm # and random up and down -25 to 25
            self._waypoints.append(copy.deepcopy(wpose))
            rospy.loginfo("self._waypoints={}".format(self._waypoints))

            # self.robot_go_left()
            # elif new_state == self._robot_state.right and last_state == self._robot_state.left:
            # elif new_state == self._robot_state.right :
            #     rospy.loginfo("[_plat_path] go right")
            #     # self._waypoints.append(copy.deepcopy(wpose))
            #     # wpose.position.x += w * 2 * dm  # and sideways (y)
            #     # wpose.position.y += w * -0 * dm  # and sideways (y)
            #     # self._waypoints.append(copy.deepcopy(wpose))
            #     self.robot_go_right()
            # else:
            #     rospy.loginfo("No new state")
            #     return False
            return True #Yes it moved the robot
        else:
            rospy.loginfo("Same state no change")
            return False #No no movment of the robot is needed
            # self._waypoints


    def _compute_path(self):
        rospy.loginfo("[_compute_path] Planing the path")
        rospy.loginfo("_compute_path _waypoints\n {}".format(self._waypoints))
        self.eff_step = rospy.get_param("valkyrie/eff_step")
        self.jump_th = rospy.get_param("valkyrie/jump_th")
        if len(self._waypoints) > 0:
            (self._robot_plan, self._robot_fraction) = self.arm.compute_cartesian_path(
                                               self._waypoints,   # waypoints to follow
                                               self.eff_step,        # eef_step
                                               self.jump_th)         # jump_threshold
            rospy.loginfo("[_compute_path] Done ")
            rospy.loginfo("The fraction:\n {}".format(self._robot_fraction))
            rospy.loginfo("The plan:\n {}".format(self._robot_plan))
        else:
            rospy.loginfo("[_compute_path] No _waypoints")
        pass

    def display_trajectory(self):
        """ Shows the planed trajectory in RViz """
        rospy.loginfo("[dispaly_trajectory]")
        robot = self._robot
        display_trajectory_publisher = self._display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(self._robot_plan)
        display_trajectory_publisher.publish(display_trajectory);


    def _exec_path(self, ask=True):
        rospy.loginfo("[_exec_path] Executing the path")
        run = True
        rospy.loginfo("[_compute_path] eff_step={}, jump_th={}".format(self.eff_step, self.jump_th))
        if False:
            self.display_trajectory()
            rospy.loginfo("[_exec_path] Run display trajectory")
            ans = str(raw_input("execute path [Yn]: "))
            if ans == 'n' or ans == 'no':
                run = False

        if not self._robot_plan is None and run:
            rospy.loginfo("arm.exec ")
            rospy.loginfo("[_exec_path] plan=\n{}".format(self._robot_plan))
            self.arm.execute(self._robot_plan, wait=True)
            rospy.loginfo("[_exec_path] done")
            self._robot_plan = None
            # rospy.loginfo("arm.stop()")
            # self.arm.stop()
        rospy.loginfo("[_exec_path] Done")
        pass

    def change_wpscale(self, scale):
        if isinstance(scale, (float|int)):
            rospy.loginfo("Changed _wpscale to {}".format(scale))
            self._wpscale = float(scale)
        else:
            rospy.logwarn("The input {} is not a valid change of scale!".format(scale))


def main():
    rospy.loginfo("Creating robot")
    r = robot()
    rospy.loginfo("Robot created [Done]")
    rospy.loginfo('Rosspin')
    rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("Start")
    main()
