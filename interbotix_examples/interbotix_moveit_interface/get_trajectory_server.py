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
MESH_OBJECT_FILENAMES = {"chair_bernhard_0146/left_leg": "furniture/env/models/assets/objects/chair_bernhard_0146/left_leg.stl",
                         "chair_bernhard_0146/right_leg": "furniture/env/models/assets/objects/chair_bernhard_0146/right_leg.stl",
                         "chair_bernhard_0146/seat": "furniture/env/models/assets/objects/chair_bernhard_0146/seat.stl",
                         "table_klubbo_0743/leg1": "furniture/env/models/assets/objects/table_klubbo_0743/leg1.stl",
                         "table_klubbo_0743/leg2": "furniture/env/models/assets/objects/table_klubbo_0743/leg2.stl",
                         "table_klubbo_0743/leg3": "furniture/env/models/assets/objects/table_klubbo_0743/leg3.stl",
                         "table_klubbo_0743/leg4": "furniture/env/models/assets/objects/table_klubbo_0743/leg4.stl",
                         "table_klubbo_0743/table_top": "furniture/env/models/assets/objects/table_klubbo_0743/table_top.stl"}


def generate_trajectory(req):
    """
    Handles service request message
    :param req: service request message
    :return: service response message
    """
    start = rospy.get_time()

    print("Generating joint trajectory for the following request... \n"
          "--- Position: [{} {} {}]\n"
          "--- Start joint values: {}\n"
          "--- Object names: {}".format(req.x, req.y, req.z, req.start_joint_values, req.object_names))

    # add mesh object to scene
    for i in range(len(req.object_names)):
        object_name = req.object_names[i]
        object_pose = req.object_poses[i]
        object_size_vector3 = req.object_sizes[i]
        object_size = (object_size_vector3.x, object_size_vector3.y, object_size_vector3.z)
        try:
            object_filename = MESH_OBJECT_FILENAMES[object_name]
            if object_name in interface.scene.get_known_object_names():
                interface.move_mesh(object_name, object_pose)
            else:
                interface.add_mesh(object_name, object_pose, object_filename, object_size)
        except KeyError:
            print("Unrecognized object name: {}".format(object_name))

    # remove objects no longer in scene
    objects_no_longer_in_scene = list(set(interface.scene.get_known_object_names()) - set(req.object_names))
    for object in objects_no_longer_in_scene:
        interface.remove_mesh(object)

    if GO_TO_START_JOINT_VALUES:
        interface.go_to_joint_state(req.start_joint_values)

    target_position = [req.x, req.y, req.z]
    trajectory_exists, plan = interface.plan_ee_position(target_position,
                                                         req.start_joint_values,
                                                         execute_plan=EXECUTE_PLAN)

    end = rospy.get_time()

    print("Trajectory exists?: {} ".format(trajectory_exists))
    print("Trajectory sent to client!")
    print("Objects in scene after trajectory update: {}".format(interface.scene.get_known_object_names()))
    print("Processed request in {} seconds".format(end - start))
    return GetTrajectoryResponse(plan.joint_trajectory, trajectory_exists)


if __name__ == "__main__":
    interface = MoveGroupPythonInteface(robot_name=ROBOT_NAME, dof=DOF)

    # remove objects in moveit scene because of previous runs
    for name in interface.scene.get_known_object_names():
        interface.scene.remove_world_object(name)

    s = rospy.Service('get_trajectory', GetTrajectory, generate_trajectory)
    print("Ready to generate trajectory.")
    rospy.spin()
