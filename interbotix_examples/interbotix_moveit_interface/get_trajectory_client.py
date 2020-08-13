#!/usr/bin/env python

from __future__ import print_function

import rospy
from interbotix_moveit_interface.srv import GetTrajectory
from geometry_msgs.msg import Pose


if __name__ == "__main__":
    x, y, z = 0.25, 0, 0.4
    start_joint_values = [0.0, -2.149075726265437e-07, -8.742276236262114e-08, -2.9802320611338473e-08, -1.4771121925605257e-07]

    object_pose = Pose()
    object_pose.position.x = 0.5
    object_pose.position.y = 0.5
    object_pose.position.z = 0
    object_pose.orientation.w = 1
    object_names = ["seat"]
    object_poses= [object_pose]

    print("Requesting trajectory...\n"
           "--- Position: [{} {} {}]\n"
          "--- Start joint values: {}\n"
          "--- Object names: {}".format(x, y, z, start_joint_values, object_names))

    rospy.wait_for_service("get_trajectory")
    try:
        get_trajectory = rospy.ServiceProxy('get_trajectory', GetTrajectory)
        resp = get_trajectory(x, y, z, start_joint_values, object_names, object_poses)

        trajectory = [point.positions for point in resp.trajectory.points]
        print("Trajectory exists? {}".format(resp.trajectory_exists))
        print("The generated trajectory is: {}".format(trajectory[-1]))
        print("The generated trajectory length is: {}".format(len(trajectory)))

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

