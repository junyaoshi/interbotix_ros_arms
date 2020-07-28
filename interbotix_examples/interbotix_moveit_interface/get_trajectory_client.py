#!/usr/bin/env python

from __future__ import print_function

import rospy
from interbotix_moveit_interface.srv import GetTrajectory


if __name__ == "__main__":
    x, y, z = 0.25, 0.0, 0.2
    start_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0]
    print("Requesting trajectory for goal [{} {} {}] "
          "and start joint values {} ".format(x, y, z, start_joint_values))

    rospy.wait_for_service("get_trajectory")
    try:
        get_trajectory = rospy.ServiceProxy('get_trajectory', GetTrajectory)
        resp = get_trajectory(x, y, z, start_joint_values)
        trajectory = []
        for point in resp.trajectory.points:
            trajectory.append(point.positions)
        print("Trajectory exists? {}".format(resp.trajectory_exists))
        print("The generated trajectory is: {}".format(trajectory))
        print("The generated trajectory length is: {}".format(len(trajectory)))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

