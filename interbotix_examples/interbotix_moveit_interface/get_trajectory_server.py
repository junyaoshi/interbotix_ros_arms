#!/usr/bin/env python

from __future__ import print_function

from moveit_python_interface import MoveGroupPythonInteface
from geometry_msgs.msg import Pose
import rospy
from interbotix_moveit_interface.srv import GetTrajectory, GetTrajectoryResponse


def generate_trajectory(req):
    print("Generating joint trajectory for [%f, %f, %f]" % (req.x, req.y, req.z))
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = req.x
    pose_goal.position.y = req.y
    pose_goal.position.z = req.z
    trajectory_exists, plan = interface.plan_ee_pose(pose_goal, execute_plan=False)
    print("Trajectory exists?: %s; "
          % (trajectory_exists))
    return GetTrajectoryResponse(plan.joint_trajectory, trajectory_exists)


if __name__ == "__main__":
    interface = MoveGroupPythonInteface(robot_name='wx200', dof=5)
    s = rospy.Service('get_trajectory', GetTrajectory, generate_trajectory)
    print("Ready to generate trajectory.")
    rospy.spin()
