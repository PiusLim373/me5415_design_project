#!/usr/bin/python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import copy
from geometry_msgs.msg import Pose, Point, Quaternion
from me5415_moveit_config.srv import *
from std_srvs.srv import SetBoolResponse
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
import time
import math
import os


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
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_opening = 0.00

        self.gripper_opening_pub = rospy.Publisher(
            "left_finger_joint_position_controller/command", Float64, queue_size=1
        )
        # rospy.Service("go_to_saved_joint", GoToSavedJoint, self.go_to_saved_joint_cb)
        # rospy.Service("move_with_joint", MoveWithJoint, self.move_with_joint_cb)
        # rospy.Service("move_with_cartesian", MoveWithCartesian, self.move_with_cartesian_cb)
        # rospy.Service("move_in_cartesian_direction", MoveInCartesianDirection, self.move_in_cartesian_direction_cb)
        # rospy.Service("get_robot_states", GetRobotStates, self.get_robot_states_cb)
        # rospy.Service("gripper_activation", SetBool, self.gripper_activation_cb)
        self.spawn_model_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        rospy.Timer(rospy.Duration(0.5), self.gripper_opening_cb)
        rospy.loginfo("Initialization Completed!")

    def gripper_opening_cb(self, event):
        data = Float64()
        data.data = self.gripper_opening
        self.gripper_opening_pub.publish(data)

    def go_to_saved_joint_cb(self, req):
        res = GoToSavedJointResponse()
        res.success = self.go_to_saved_joint(req.saved_joint)
        return res

    def move_with_joint_cb(self, req):
        res = MoveWithJointResponse()
        if len(req.joint_angles) != 6:
            rospy.logerr("6 joint angles are expected!")
            res.success = False
            return res
        res.success = self.move_with_joint(req.joint_angles)
        return res

    def move_with_cartesian_cb(self, req):
        res = MoveWithCartesianResponse()
        print(req)
        res.success = self.move_with_cartesian(req.pose_goal.pose)
        return res

    def move_in_cartesian_direction_cb(self, req):
        res = MoveInCartesianDirectionResponse()
        if not req.direction in ["x", "y", "z"]:
            rospy.logerr("Only x, y and z direction is allowed!")
            res.success = False
            return res
        if req.distance == 0.0:
            rospy.logerr("distance cant be zero")
            res.success = False
            return res
        res.success = self.move_in_cartesian_direction(req.direction, req.distance)
        return res

    def get_robot_states_cb(self, req):
        print("====== joint states ======")
        print(self.robot.get_current_state())
        print("====== cartesian states ======")
        print(self.move_group.get_current_pose())
        res = GetRobotStatesResponse()
        res.joint_angles = list(self.robot.get_current_state().joint_state.position)
        res.pose = self.move_group.get_current_pose().pose
        res.success = True
        return res

    def gripper_activation_cb(self, req):
        # set true to close, false to open
        res = SetBoolResponse()
        data = Float64()
        if req.data:
            self.open_gripper()
        else:
            self.close_gripper()
        res.success = True
        return res

    def open_gripper(self, opening=0.03):
        self.gripper_opening = opening
        time.sleep(1)
        return True

    def close_gripper(self, opening=0.00):
        self.gripper_opening = opening
        time.sleep(1)
        return True

    def go_to_saved_joint(self, saved_joint):
        joint_goal = self.move_group.set_named_target(saved_joint)
        result = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return result

    def move_with_joint(self, joint_goal):
        result = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return result

    def rotate_specific_joint(self, joint, magnitude):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[joint] += magnitude
        result = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return result

    def generate_waypoints_with_step(self, start_pose, end_pose, use_curr_quaternion=False, distance_per_step=0.05):

        step_size = int(
            max(
                [
                    math.ceil(abs(end_pose.position.x - start_pose.position.x) / distance_per_step),
                    math.ceil(abs(end_pose.position.y - start_pose.position.y) / distance_per_step),
                    math.ceil(abs(end_pose.position.z - start_pose.position.z) / distance_per_step),
                ]
            )
        )
        waypoints = []

        incremental = Point(
            (end_pose.position.x - start_pose.position.x) / step_size,
            (end_pose.position.y - start_pose.position.y) / step_size,
            (end_pose.position.z - start_pose.position.z) / step_size,
        )
        print(end_pose)
        for i in range(1, step_size + 1):
            temp_waypoint = Pose()
            temp_waypoint.position.x = start_pose.position.x + i * incremental.x
            temp_waypoint.position.y = start_pose.position.y + i * incremental.y
            temp_waypoint.position.z = start_pose.position.z + i * incremental.z
            if use_curr_quaternion:
                temp_waypoint.orientation.x = start_pose.orientation.x
                temp_waypoint.orientation.y = start_pose.orientation.y
                temp_waypoint.orientation.z = start_pose.orientation.z
                temp_waypoint.orientation.w = start_pose.orientation.w
            else:
                temp_waypoint.orientation.x = end_pose.orientation.x
                temp_waypoint.orientation.y = end_pose.orientation.y
                temp_waypoint.orientation.z = end_pose.orientation.z
                temp_waypoint.orientation.w = end_pose.orientation.w
            waypoints.append(temp_waypoint)

        return waypoints

    def move_with_cartesian_no_iteration(self, pose_goal):
        self.move_group.clear_pose_targets()
        print(pose_goal)
        self.move_group.set_pose_target(pose_goal)
        result = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return result

    def move_with_cartesian(self, pose_goal_list):
        use_curr_quaternion = False
        if len(pose_goal_list) == 3:
            use_curr_quaternion = True
        pose_goal = self.list_to_pose(pose_goal_list)
        print(self.move_group.get_current_pose().pose)
        pose_list = self.generate_waypoints_with_step(
            start_pose=self.move_group.get_current_pose().pose,
            end_pose=pose_goal,
            use_curr_quaternion=use_curr_quaternion,
        )
        for x in pose_list:
            print(x.position.x, x.position.y, x.position.z)
        (path, frac) = self.move_group.compute_cartesian_path(pose_list, 0.01, 0.0)
        print(f"frac: {frac}")
        result = self.move_group.execute(path, wait=True)
        self.move_group.stop()
        return result

        # for i in range(10):
        #     (traj, frac) = self.move_group.compute_cartesian_path(pose_list, 0.01, 0.0)
        #     print(f"frac: {frac}")
        #     # print(traj)
        #     shoulder_pan_start = traj.joint_trajectory.points[0].positions[0]
        #     shoulder_pan_final = traj.joint_trajectory.points[-1].positions[0]
        #     if abs(shoulder_pan_final - shoulder_pan_start) > 0.5 * math.pi:
        #         rospy.logwarn("Joint flip detected, to retry")
        #         continue
        #     else:
        #         print("inside go")
        #         result = self.move_group.execute(traj, wait=True)
        #         self.move_group.stop()
        #         print(result)
        #         return result
        # rospy.logwarn("All 10 trial result in joint flip")
        # return False

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

    def list_to_pose(self, data):
        temp = Pose()
        temp.position.x = data[0]
        temp.position.y = data[1]
        temp.position.z = data[2]
        if len(data) != 7:
            temp.orientation = self.move_group.get_current_pose().pose.orientation
        else:
            temp.orientation.x = data[3]
            temp.orientation.y = data[4]
            temp.orientation.z = data[5]
            temp.orientation.w = data[6]
        return temp


def run_service():
    rospy.init_node("robot_controller")
    robot = MoveGroupPythonInteface()
    rospy.spin()


def normal_tray_transfer(food_coor, placing_coor, gripper_opening=0.03):
    print("source table homing")
    if ROBOT.go_to_saved_joint("source_home"):
        print("moving to meatball tray")
        if ROBOT.move_with_cartesian(food_coor):
            print("opening gripper")
            ROBOT.open_gripper(gripper_opening)
            print("going down")
            if ROBOT.move_cartesian_direction("z", -0.1):
                print("closing gripper")
                ROBOT.close_gripper()
                print("going up")
                if ROBOT.move_cartesian_direction("z", 0.1):
                    print("source table homing")
                    if ROBOT.go_to_saved_joint("source_home"):
                        print("assemble table homing")
                        if ROBOT.go_to_saved_joint("assemble_home"):
                            print("moving to plate")
                            if ROBOT.move_with_cartesian(placing_coor):
                                print("going down")
                                if ROBOT.move_cartesian_direction("z", -0.1):
                                    print("opening gripper")
                                    ROBOT.open_gripper(gripper_opening)
                                    print("going up")
                                    if ROBOT.move_cartesian_direction("z", 0.1):
                                        print("closing gripper")
                                        ROBOT.close_gripper()
                                        print("assemble table homing")
                                        if ROBOT.go_to_saved_joint("assemble_home"):
                                            print("all done!")
                                            return True
    return False


def retry_fail_placement(placing_coor, gripper_opening=0.03):
    if ROBOT.go_to_saved_joint("assemble_home"):
        print("moving to plate")
        if ROBOT.move_with_cartesian(placing_coor):
            print("going down")
            if ROBOT.move_cartesian_direction("z", -0.1):
                print("opening gripper")
                ROBOT.open_gripper(gripper_opening)
                print("going up")
                if ROBOT.move_cartesian_direction("z", 0.1):
                    print("closing gripper")
                    ROBOT.close_gripper()
                    print("assemble table homing")
                    if ROBOT.go_to_saved_joint("assemble_home"):
                        print("all done!")
                        return True
    return False


def transfer_juice():
    if ROBOT.go_to_saved_joint("source_home"):
        if ROBOT.go_to_saved_joint("source_juice_home"):
            if ROBOT.move_with_cartesian(ITEM_POSES["juice"]):
                ROBOT.open_gripper()
                if ROBOT.move_cartesian_direction("x", 0.1):
                    ROBOT.close_gripper()
                    if ROBOT.move_cartesian_direction("z", 0.2):
                        if ROBOT.go_to_saved_joint("assemble_juice_b_home"):
                            if ROBOT.move_with_cartesian(ITEM_POSES["cup_B"]):
                                if ROBOT.rotate_specific_joint(5, 0.785398):
                                    time.sleep(1)
                                    if ROBOT.rotate_specific_joint(5, -0.785398):
                                        if ROBOT.go_to_saved_joint("assemble_juice_a_home"):
                                            if ROBOT.move_with_cartesian(ITEM_POSES["cup_A"]):
                                                if ROBOT.rotate_specific_joint(5, 0.785398):
                                                    time.sleep(1)
                                                    if ROBOT.rotate_specific_joint(5, -0.785398):
                                                        if ROBOT.go_to_saved_joint("source_juice_home"):
                                                            if ROBOT.move_with_cartesian(ITEM_POSES["juice_putback"]):
                                                                if ROBOT.move_cartesian_direction("x", 0.1):
                                                                    ROBOT.open_gripper()
                                                                    if ROBOT.move_cartesian_direction("x", -0.1):
                                                                        if ROBOT.go_to_saved_joint("source_home"):
                                                                            return True
    return False


def spawn_model(unique_name, sdf_path, spawn_pose):
    with open(sdf_path, "r") as sdf_file:
        sdf_content = sdf_file.read()

    service_data = SpawnModelRequest()
    service_data.model_name = unique_name
    service_data.model_xml = sdf_content
    service_data.robot_namespace = ""
    service_data.initial_pose = spawn_pose
    service_data.reference_frame = "world"

    result = ROBOT.spawn_model_service.call(service_data)
    if result.success:
        print("successfully spawned model!")
        return True
    return False


def debug():
    ROBOT.close_gripper()
    if ROBOT.go_to_saved_joint("source_juice_home"):
        if ROBOT.move_with_cartesian(ITEM_POSES["juice_putback"]):
            if ROBOT.move_cartesian_direction("x", 0.1):
                ROBOT.open_gripper()
                if ROBOT.move_cartesian_direction("x", -0.1):
                    if ROBOT.go_to_saved_joint("source_home"):
                        pass


def transfer_food_flow():
    # broccoli
    normal_tray_transfer(ITEM_POSES["brocolli"], ITEM_POSES["bowl_A"])
    normal_tray_transfer(ITEM_POSES["brocolli"], ITEM_POSES["bowl_B"])
    # noodles (to-do)
    # meatball
    normal_tray_transfer(ITEM_POSES["meatball"], ITEM_POSES["container_1A"])
    normal_tray_transfer(ITEM_POSES["meatball"], ITEM_POSES["container_1B"])
    # egg
    normal_tray_transfer(ITEM_POSES["egg"], ITEM_POSES["plate_A"], gripper_opening=0.06)
    normal_tray_transfer(ITEM_POSES["egg"], ITEM_POSES["plate_B"], gripper_opening=0.06)
    transfer_juice()


def spawn_food():
    # broccolli
    spawn_pose = Pose(Point(x=0.43, y=0.19, z=0.76), Quaternion(x=0, y=0, z=0, w=1))
    spawn_model("broccolli6", "/home/sesto/.gazebo/models/broccoli_simplified/model.sdf", spawn_pose)
    # meatball
    spawn_pose = Pose(Point(x=0.43, y=-0.21, z=0.74), Quaternion(x=0, y=0, z=0, w=1))
    spawn_model("meatball6", "/home/sesto/.gazebo/models/meatball/model.sdf", spawn_pose)

    # egg
    spawn_pose = Pose(Point(x=0.43, y=-0.41, z=0.75), Quaternion(x=0, y=0, z=0, w=1))
    spawn_model("egg2", "/home/sesto/.gazebo/models/egg/model.sdf", spawn_pose)


if __name__ == "__main__":
    rospy.init_node("robot_controller")
    ITEM_POSES = rospy.get_param("~poses", None)
    print(ITEM_POSES)
    ROBOT = MoveGroupPythonInteface()

    normal_tray_transfer(ITEM_POSES["egg"], ITEM_POSES["plate_A"], gripper_opening=0.06)
    # retry_fail_placement(ITEM_POSES["plate_B"], gripper_opening=0.06)

    # spawn_pose = Pose(Point(x=0.43, y=-0.41, z=0.75),Quaternion(x=0, y=0, z=0, w=1))
    # spawn_model("egg0", '/home/sesto/.gazebo/models/egg/model.sdf', spawn_pose)

    """
        Usage
        1.  Run the normal_tray_transfer, fill in food, placing coordinate and gripper opening for bigger items, eg: egg
            Some sample is included in transfer_food_flow()
        2.  Sometimes the planner will fail if the robot eef is wiggling (idk how to fix this physics yet..), normally will fail at assemble station
            Solution is to wait abit for the joint to stable down, then run the retry_fail_placement() function
        3.  If transfer to assemble station A is completed, use the spawn_model function to spawn the model again, since can only pick one at a time
            The spawn name must be unique, model_xml is the absolute path to the model.sdf in ~/.gazebo/models/xxx/, gripper_opening is same as above
            The spawn pose is the same as what is defined in the first element of each type in me5415.world
    
    
    
    """
