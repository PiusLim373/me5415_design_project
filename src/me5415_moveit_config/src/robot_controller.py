#!/usr/bin/python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import copy
from geometry_msgs.msg import Pose


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
        )

        print("done declaring pub")

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_saved_joint(self, saved_joint):
        joint_goal = self.move_group.set_named_target(saved_joint)
        result = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return result

    def move_with_joint(self, joint_goal):
        result = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return result

    def move_with_cartesian(self, pose_goal):
        self.move_group.set_pose_target(pose_goal)
        result = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return result

    def move_cartesian_direction(self, axis, distance):
        waypoints = []
        curr_pose = self.move_group.get_current_pose().pose
        if axis == "x":
            curr_pose.position.x += distance
        elif axis == "y":
            curr_pose.position.y += distance
        elif axis == "z":
            curr_pose.position.z += distance
        waypoints.append(copy.deepcopy(curr_pose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )  # fraction is percentage viable, 0.95 means can reach 95% of the cartesian pose requested
        if fraction == 1:
            result = self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            return result

    def get_status(self):
        print("====== joint states ======")
        print(self.robot.get_current_state())
        print("====== cartesian states ======")
        print(self.move_group.get_current_pose())


def main():
    rospy.init_node("robot_controller")
    robot = MoveGroupPythonInteface()

    input("press enter go to source table via saved joint goal")
    robot.go_to_saved_joint("source_home")

    input("press enter go to assembly table via saved joint goal")
    robot.go_to_saved_joint("assemble_home")

    input("press enter to move via joint")
    robot.move_with_joint([1.0, -2.5, 1, -1, -1.5, 0])  # 6 joint value in radian

    input("press enter to move via joint")
    robot.move_with_joint([0.75, -2, 2, -1.5, -1.5, 0])  # 6 joint value in radian

    print(robot.get_status())
    input("press enter to move via cartesian")  # supply a pose, use ik to calculate the joint angle and move
    pose_goal = Pose()
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0
    pose_goal.position.x = 0.2
    pose_goal.position.y = 0.15
    pose_goal.position.z = 1.0
    robot.move_with_cartesian(pose_goal)

    input("press enter to move via in specific cartesian direction")
    robot.move_cartesian_direction("z", -0.1)  # move in a specific cartesian direction, wrt to base_link

    input("press enter to move via in specific cartesian direction")
    robot.move_cartesian_direction("x", 0.2)  # move in a specific cartesian direction, wrt to base_link


if __name__ == "__main__":
    main()
