#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from shape_msgs.msg import Mesh, MeshTriangle
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInteface(object):
    def __init__(self, robot_name, dof):
        super(MoveGroupPythonInteface, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)

        # Get the name of the robot - this will be used to properly define the end-effector link when adding a box
        self.robot_name = robot_name

        # Get the dof of the robot - this will make sure that the right number of joints are controlled
        self.dof = dof

        group_name = "interbotix_arm"
        robot_description = "wx200/robot_description"

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander(robot_description=robot_description, ns=self.robot_name)

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface(ns=self.robot_name)

        # fix the incorrect publisher topic names in moveit_commander.PlanningSceneInterface
        scene._pub_co = rospy.Publisher('/' + robot_name + '/collision_object',
                                        moveit_msgs.msg.CollisionObject,
                                        queue_size=100)
        scene._pub_aco = rospy.Publisher('/' + robot_name + '/attached_collision_object',
                                         moveit_msgs.msg.AttachedCollisionObject,
                                         queue_size=100)

        # Instantiate a `MoveGroupCommander`_ object. This object is an interface to one group of joints.
        # This interface can be used to plan and execute motions on the Interbotix Arm:
        group = moveit_commander.MoveGroupCommander(name=group_name,
                                                    robot_description=robot_description,
                                                    ns=self.robot_name)

        # We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.meshes = {}

    def go_to_joint_state(self, joint_goal):
        """
        Planning to a Joint Goal
        """
        group = self.group

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

    def plan_ee_position(self, target_position, start_joint_values, execute_plan=False):
        """
        Planning to a Target Position
        :param target_position: [x, y, z]
        :param start_joint_values: n-element list, where n is the number of joints
        :param execute_plan: whether to execute the plan in Rviz after planning
        :return:
          trajectory_exists: whether trajectory to reach pose goal exists
          plan: the generated trajectory plan, trajectory points will be an empty list if trajectory_exists=False
        """
        start = rospy.get_time()
        group = self.group

        # set the initial state of robot in the generated trajectory plan
        if start_joint_values is not None:
            start_state = self.robot.get_current_state()
            start_position = list(start_state.joint_state.position)
            start_position[:len(start_joint_values)] = start_joint_values
            start_state.joint_state.position = tuple(start_position)
            group.set_start_state(start_state)

        group.set_position_target(target_position)

        # Now, we call the planner to compute the plan and execute it.
        plan = group.plan()
        trajectory_exists = not not plan.joint_trajectory.points
        if execute_plan:
            group.execute(plan, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        end = rospy.get_time()
        print("Calculated trajectory in {} seconds".format(end - start))

        return trajectory_exists, plan

    def plan_ee_pose(self, target_pose, start_joint_values, execute_plan=False):
        """
        Planning to a Target Pose
        :param target_pose: a geometry_msgs.msg.Pose object
        :param start_joint_values: n-element list, where n is the number of joints
        :param execute_plan: whether to execute the plan in Rviz after planning
        :return:
          trajectory_exists: whether trajectory to reach pose goal exists
          plan: the generated trajectory plan, trajectory points will be an empty list if trajectory_exists=False
        """
        start = rospy.get_time()
        group = self.group

        # set the initial state of robot in the generated trajectory plan
        if start_joint_values is not None:
            start_state = self.robot.get_current_state()
            start_position = list(start_state.joint_state.position)
            start_position[:len(start_joint_values)] = start_joint_values
            start_state.joint_state.position = tuple(start_position)
            group.set_start_state(start_state)

        group.set_pose_target(target_pose)

        # Now, we call the planner to compute the plan and execute it.
        plan = group.plan()
        trajectory_exists = not not plan.joint_trajectory.points
        if execute_plan:
            group.execute(plan, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        end = rospy.get_time()
        print("Calculated trajectory in {} seconds".format(end - start))

        return trajectory_exists, plan

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z += scale * 0.1  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.1  # Third move down (z)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()
            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_mesh(self, name, pose, filename, size=(1, 1, 1), timeout=0.5):

        # convert pose to pose stamped
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = self.planning_frame

        start = rospy.get_time()
        mesh = self.__add_mesh(name, pose_stamped, filename, size)
        self.meshes[name] = mesh
        end = rospy.get_time()
        print("Add mesh time: {}".format(end - start))

        # wait until the object info is published
        start = rospy.get_time()
        seconds = rospy.get_time()
        is_known = False
        while (seconds - start < timeout) and not rospy.is_shutdown():

            # Test if the box is in the scene.
            is_known = name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if is_known:
                break

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        if not is_known:
            print("Failed to load object: {} within {} seconds".format(name, timeout))
        else:
            print("Loaded object in {} seconds".format(seconds - start))

    def move_mesh(self, name, pose):

        # convert pose to pose stamped
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = self.planning_frame

        start = rospy.get_time()
        self.__move_mesh(name, pose_stamped)
        end = rospy.get_time()
        print("Time spent moving mesh object: {}".format(end - start))

    def __add_mesh(self, name, pose, filename, scale=(1, 1, 1)):
        """
        This method is based on the __make_mesh() method of the moveit_commander.PlanningSceneInterface class
        :return:
            mesh: the mesh message corresponding to the mesh object
        """

        co = moveit_msgs.msg.CollisionObject()

        # pyassimp scene loading
        if pyassimp is False:
            raise moveit_commander.exception.MoveItCommanderException(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise moveit_commander.exception.MoveItCommanderException("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise moveit_commander.exception.MoveItCommanderException("There are no faces in the mesh")

        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header = pose.header

        # populating mesh message
        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, '__len__'):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, 'indices'):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise moveit_commander.exception.MoveItCommanderException(
                "Unable to build triangles from mesh due to mesh object structure")
        for vertex in scene.meshes[0].vertices:
            point = geometry_msgs.msg.Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)

        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        self.scene._pub_co.publish(co)

        return mesh

    def __move_mesh(self, name, pose):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.MOVE
        co.id = name
        co.header = pose.header

        if name not in self.meshes.keys():
            raise ValueError('Unrecognized name while moving mesh object: {}. '
                             'This object is not in the scene.'.format(name))
        mesh = self.meshes[name]

        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        self.scene._pub_co.publish(co)

    def remove_mesh(self, name):
        self.scene.remove_world_object(name)

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot_name = self.robot_name
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = robot_name + "/ee_gripper_link"
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.025, 0.025, 0.05))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Arm's wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Interbotix
        ## robot, we set ``grasping_group = 'interbotix_gripper'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'interbotix_gripper'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
    try:
        print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
        raw_input()
        tutorial = MoveGroupPythonInteface(robot_name="wx200", dof=5)

        print "============ Press `Enter` to add a box to the planning scene ..."
        raw_input()
        tutorial.add_box()

        print "============ Press `Enter` to attach a Box to the Interbotix robot ..."
        raw_input()
        tutorial.attach_box()

        print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        raw_input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)

        print "============ Press `Enter` to detach the box from the Interbotix robot ..."
        raw_input()
        tutorial.detach_box()

        print "============ Press `Enter` to remove the box from the planning scene ..."
        raw_input()
        tutorial.remove_box()

        print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
