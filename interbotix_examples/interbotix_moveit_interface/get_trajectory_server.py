#!/usr/bin/env python

from __future__ import print_function

from moveit_python_interface import MoveGroupPythonInteface
from geometry_msgs.msg import Pose
import rospy
from interbotix_moveit_interface.srv import GetTrajectory, GetTrajectoryResponse

# flags
GO_TO_START_JOINT_VALUES = False           # whether to use execute a plan to go to start joint values
EXECUTE_PLAN = False                       # whether to execute the plan generated for reaching pose goal
ROBOT_NAME = 'wx200'                       # name of the robot
DOF = 5                                    # robot's degrees of freedom


def generate_trajectory(req):
    """
    Handles service request message
    :param req: service request message
    :return: service response message
    """
    print("Generating joint trajectory for [{} {} {}] "
          "and start joint values {}".format(req.x, req.y, req.z, req.start_joint_values))

    # initialize pose goal
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0  # we need to set a default orientation: (0, 0, 0, 1) in quaternion
    pose_goal.position.x = req.x
    pose_goal.position.y = req.y
    pose_goal.position.z = req.z

    if GO_TO_START_JOINT_VALUES:
        interface.go_to_joint_state(req.start_joint_values)

    trajectory_exists, plan = interface.plan_ee_pose(pose_goal, req.start_joint_values, execute_plan=EXECUTE_PLAN)

    print("Trajectory exists?: {} ".format(trajectory_exists))
    print("Trajectory sent to client!")
    return GetTrajectoryResponse(plan.joint_trajectory, trajectory_exists)


if __name__ == "__main__":
    interface = MoveGroupPythonInteface(robot_name=ROBOT_NAME, dof=DOF)
    s = rospy.Service('get_trajectory', GetTrajectory, generate_trajectory)
    print("Ready to generate trajectory.")
    rospy.spin()
