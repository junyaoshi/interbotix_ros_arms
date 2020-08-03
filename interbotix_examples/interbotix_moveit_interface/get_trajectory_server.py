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

# mesh object path dictionary
MESH_OBJECT_FILENAMES = {"left_leg": "furniture/env/models/assets/objects/chair_bernhard_0146/left_leg.stl",
                         "right_leg": "furniture/env/models/assets/objects/chair_bernhard_0146/right_leg.stl",
                         "seat": "furniture/env/models/assets/objects/chair_bernhard_0146/seat.stl"}


def generate_trajectory(req):
    """
    Handles service request message
    :param req: service request message
    :return: service response message
    """
    print("Generating joint trajectory for the following request... \n"
          "Position: [{} {} {}]\n "
          "Start joint values: {}\n"
          "Object names: {}".format(req.x, req.y, req.z, req.start_joint_values, req.object_names))

    # initialize pose goal
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0  # we need to set a default orientation: (0, 0, 0, 1) in quaternion
    pose_goal.position.x = req.x
    pose_goal.position.y = req.y
    pose_goal.position.z = req.z

    # add mesh object to scene
    for i in range(len(req.object_names)):
        object_name = req.object_names[i]
        object_pose = req.object_poses[i]
        try:
            object_filename = MESH_OBJECT_FILENAMES[object_name]
            if object_name in objects_in_scene:
                interface.remove_mesh(object_name)
            else:
                objects_in_scene.append(object_name)
            interface.add_mesh(object_name, object_pose, object_filename)
        except KeyError:
            print("Unrecognized object name: {}".format(object_name))

    # remove objects no longer in scene
    objects_no_longer_in_scene = list(set(objects_in_scene) - set(req.object_names))
    for object in objects_no_longer_in_scene:
        interface.remove_mesh(object)
        objects_in_scene.remove(object)

    if GO_TO_START_JOINT_VALUES:
        interface.go_to_joint_state(req.start_joint_values)

    trajectory_exists, plan = interface.plan_ee_pose(pose_goal, req.start_joint_values, execute_plan=EXECUTE_PLAN)

    print("Trajectory exists?: {} ".format(trajectory_exists))
    print("Trajectory sent to client!")
    print("Objects in scene after trajectory update: {}".format(objects_in_scene))
    return GetTrajectoryResponse(plan.joint_trajectory, trajectory_exists)


if __name__ == "__main__":
    interface = MoveGroupPythonInteface(robot_name=ROBOT_NAME, dof=DOF)
    objects_in_scene = []
    s = rospy.Service('get_trajectory', GetTrajectory, generate_trajectory)
    print("Ready to generate trajectory.")
    rospy.spin()
