#!/usr/bin/env python2.7
import numpy as np
from math import pi, sqrt
import sys
import copy
import random
import copy

import rospy
import tf
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from moveit_msgs.msg import DisplayTrajectory, JointConstraint, Constraints
from std_msgs.msg import String, Float64, Bool, Header
from std_srvs.srv import Trigger, TriggerResponse, Empty
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from text_msgs.srv import *
from text_msgs.msg import *

# FSM ID
STATE_UNKOWN = -1
STATE_IDLE = 0
STATE_RECOGNITION = 1
STATE_POSE_ESTIMATION = 2
STATE_PICKING = 3
STATE_APRILTAG_LOC = 4
STATE_PLACING = 5
STATE_GO_HOME = 6
STATE_IMAGE_COLLECTION = 7
STATE_PLAN_ERROR = 8
STATE_RANDOM_PICKING = 10

# Initial robot pose
JOINTS_PICKING_HOME = [3.1415926, -2.21656, 1.81009, -1.57079, -1.57079, 0.0]
JOINTS_PLACING_HOME = [2.35891, -1.62645, 1.4686, -1.37149, -1.57006, -0.78227]
JOINTS_SCANNING = [2.78776478767395, -2.512611214314596, 2.008678436279297,
                   -1.7134316603290003, -1.3840788046466272, -0.27180654207338506]

# Placing related stuff
LARGE_CUBOID_PLACING_OFFSET = (+0.03, -0.14)
MEDIUM_CUBOID_PLACING_OFFSET = (+0.13, -0.14)
CYLINDER_PLACING_OFFSET = (+0.23, -0.14)
PLACING_TAG_ID = 1
FIXED_PLACING_HEIGHT = 0.23


class FSM(object):
    """docstring for FSM"""
    def __init__(self):
        self.next_state = STATE_UNKOWN
        self.cur_state = STATE_UNKOWN
        self.past_state = STATE_UNKOWN

        self.target_pose = None
        self.holding_obj = None
        self.object_found_num = None
        self.cnt_large_cuboid = 0
        self.cnt_medium_cuboid = 0
        self.cnt_cylinder = 0

        # Add CollisionObject
        self.update_collision_object()

        # Set planning constraints
        robot_js = (self.robot.get_current_state()).joint_state
        cons = Constraints()
        for i in range(len(robot_js.name)):
            # Skip the second and third joints
            if i == 1 or i == 2:
                continue
            # Limit:   [position - tolerance_below, position + tolerance_above]
            jcm = JointConstraint()
            jcm.joint_name = robot_js.name[i]
            jcm.position = JOINTS_PICKING_HOME[i]
            jcm.tolerance_below = pi/2
            jcm.tolerance_above = pi/2
            jcm.weight = 1.0 - 0.1 * i
            cons.joint_constraints.append(copy.deepcopy(jcm))
        self.move_group.set_path_constraints(cons)
        # self.move_group.set_planning_time(10)

        # TF listener
        self.tf_listener = tf.TransformListener()

        # ROS publisher & subscriber & service
        self.pub_target_pose = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.home_srv = rospy.Service("m_home", Trigger, self.home_cb)
        self.image_collect_srv = rospy.Service("m_image_collect", Trigger, self.image_collect_cb)
        self.trigger_srv = rospy.Service("m_pick", Trigger, self.pick_cb)
        self.loc_srv = rospy.Service("m_localize", Trigger, self.loc_cb)
        self.calibrate_srv = rospy.Service("m_calibrate", Trigger, self.calibrate_cb)
        self.pose_srv = rospy.Service("m_pose", Trigger, self.pose_cb)


    ############################ FSM Transition callback ############################
    def calibrate_cb(self, req):
        self.next_state = STATE_CALIBRATION
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            success, message = (False, "Request denied")

    def loc_cb(self, req):
        self.next_state = STATE_APRILTAG_LOC
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")

    def image_collect_cb(self, req):
        self.next_state = STATE_IMAGE_COLLECTION
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")

    def home_cb(self, req):
        self.next_state = STATE_GO_HOME
        if self.cur_state != self.next_state:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            return TriggerResponse(success=False, message="Request denied")
    
    def pose_cb(self, req):
        self.next_state = STATE_POSE_ESTIMATION
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")

    def pick_cb(self, req):
        self.next_state = STATE_PICKING
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")


    ############################ FSM State main task ############################
    def process(self):
        # Update FSM state
        self.past_state = self.cur_state
        self.cur_state = self.next_state

        # if self.cur_state != self.past_state:
        #     if self.cur_state == STATE_GO_HOME:
        #         self.move_group.clear_pose_targets()
        #         self.move_group.set_joint_value_target(JOINTS_PICKING_HOME)
        #         self.move_group.plan()
        #         self.move_group.go(wait=True)
        #         rospy.loginfo('Ready to pick')

        #     elif self.cur_state == STATE_POSE_ESTIMATION:
        #         self.move_group.clear_pose_targets()
        #         self.move_group.set_joint_value_target(JOINTS_SCANNING)
        #         self.move_group.plan()
        #         self.move_group.go(wait=True)
        #         try:
        #             rospy.wait_for_service('/get_pick_pose_service/get_pose', timeout=3)
        #             pose_srv = rospy.ServiceProxy('/get_pick_pose_service/get_pose', get_pick_pose)
        #             result = pose_srv()
        #             if result.status == 'success':
        #                 self.target_pose = result.pick_pose
        #                 self.target_pose.pose.position.z += 0.20
        #                 self.holding_obj = result.obj_str
        #                 self.object_found_num = result.cluster_num
        #                 self.next_state = STATE_PICKING
        #                 print "Get object pose"
        #             else:
        #                 self.next_state = STATE_PLAN_ERROR
        #         except rospy.ROSException:
        #             self.next_state = STATE_PLAN_ERROR

        #     elif self.cur_state == STATE_PICKING:
        #         try:
        #             # self.target_pose = rospy.wait_for_message('/obj_pose', PoseStamped, timeout=3)
        #             # Go to 5 cm top of placing pose
        #             pre_target_pose = copy.deepcopy(self.target_pose)
        #             pre_target_pose.pose.position.z += 0.1
        #             self.move_group.set_pose_target(pre_target_pose.pose)
        #             self.pub_target_pose.publish(pre_target_pose)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":    # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             rospy.sleep(2)
        #             self.move_group.go(wait=True)
        #             # Go to placing pose
        #             self.pub_target_pose.publish(self.target_pose)
        #             self.move_group.set_pose_target(self.target_pose.pose)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":    # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             rospy.sleep(2)
        #             self.move_group.go(wait=True)
        #             # Close gripper
        #             rospy.wait_for_service('/robotiq_finger_control_node/close_gripper', timeout=3)
        #             close_action = rospy.ServiceProxy('/robotiq_finger_control_node/close_gripper', Empty)
        #             close_action()
        #             rospy.sleep(1)
        #             # Go back to 5 cm top of placing pose
        #             post_target_pose = pre_target_pose
        #             self.move_group.set_pose_target(post_target_pose.pose)
        #             self.pub_target_pose.publish(post_target_pose)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":  # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             self.move_group.go(wait=True)
        #             # Record current pick object
        #             if self.holding_obj == "large_cuboid":
        #                 self.cnt_large_cuboid += 1
        #             elif self.holding_obj == "medium_cuboid":
        #                 self.cnt_medium_cuboid += 1
        #             elif self.holding_obj == "cylinder":
        #                 self.cnt_cylinder += 1
        #         except rospy.ROSException as e:
        #             rospy.logerr('No target pose to conduct picking task.', e)
        #             self.next_state = STATE_PLAN_ERROR
        #             return

        #         self.next_state = STATE_APRILTAG_LOC

        #     elif self.cur_state == STATE_RANDOM_PICKING:
        #         while not rospy.is_shutdown():
        #             p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #             p.pose.position.x = random.uniform(-0.5, -0.3)
        #             p.pose.position.y = random.uniform(-0.25, 0.25)
        #             p.pose.position.z = 0.28
        #             q = tf.transformations.quaternion_from_euler(pi, pi/2, random.uniform(-pi/3, pi/3))
        #             p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #             self.move_group.set_pose_target(p.pose)
        #             self.pub_target_pose.publish(p)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":    # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             else:
        #                 rospy.sleep(1)
        #                 self.move_group.go(wait=True)

        #     elif self.cur_state == STATE_APRILTAG_LOC:
        #         # x, y, z = [-0.4, 0.2, 0.5]
        #         self.move_group.clear_pose_targets()
        #         self.move_group.set_joint_value_target(JOINTS_PLACING_HOME)
        #         self.move_group.plan()
        #         self.move_group.go(wait=True)
                
        #         # Apriltag detection
        #         p = None
        #         msg = None
        #         try:
        #             msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout=3)
        #             flag_found_tag = False
        #             for i in range(len(msg.detections)):
        #                 if msg.detections[i].id == PLACING_TAG_ID:
        #                     # Calculate the position where the object to place
        #                     tmp_p = copy.deepcopy(msg.detections[i].pose)
        #                     flag_found_tag = True
        #                     quat = [tmp_p.pose.orientation.x, tmp_p.pose.orientation.y, tmp_p.pose.orientation.z, tmp_p.pose.orientation.w]
        #                     rot_mat = tf.transformations.quaternion_matrix(quat)[0:3, 0:3]
        #                     if self.holding_obj == "large_cuboid":
        #                         trans = np.atleast_2d(np.array((LARGE_CUBOID_PLACING_OFFSET[0], LARGE_CUBOID_PLACING_OFFSET[1], 0))).T
        #                         place_offset = np.dot(rot_mat, trans)
        #                         tmp_p.pose.position.x += place_offset[0][0]
        #                         tmp_p.pose.position.y += place_offset[1][0]
        #                         tmp_p.pose.position.z += place_offset[2][0]
        #                         rospy.loginfo("place to large_cuboid: {}, {}".format(tmp_p.pose.position.x, tmp_p.pose.position.y))
        #                         p = self.tf_listener.transformPose('base_link', tmp_p)
        #                     elif self.holding_obj == "medium_cuboid":
        #                         trans = np.atleast_2d(np.array((MEDIUM_CUBOID_PLACING_OFFSET[0], MEDIUM_CUBOID_PLACING_OFFSET[1], 0))).T
        #                         place_offset = np.dot(rot_mat, trans)
        #                         tmp_p.pose.position.x += place_offset[0][0]
        #                         tmp_p.pose.position.y += place_offset[1][0]
        #                         tmp_p.pose.position.z += place_offset[2][0]
        #                         rospy.loginfo("place to medium_cuboid: {}, {}".format(tmp_p.pose.position.x, tmp_p.pose.position.y))
        #                         p = self.tf_listener.transformPose('base_link', tmp_p)
        #                     elif self.holding_obj == "cylinder":
        #                         trans = np.atleast_2d(np.array((CYLINDER_PLACING_OFFSET[0], CYLINDER_PLACING_OFFSET[1], 0))).T
        #                         place_offset = np.dot(rot_mat, trans)
        #                         tmp_p.pose.position.x += place_offset[0][0]
        #                         tmp_p.pose.position.y += place_offset[1][0]
        #                         tmp_p.pose.position.z += place_offset[2][0]
        #                         rospy.loginfo("place to cylinder: {}, {}".format(tmp_p.pose.position.x, tmp_p.pose.position.y))
        #                         p = self.tf_listener.transformPose('base_link', tmp_p)
        #                     else:
        #                         rospy.logerr('Object picking failed')
        #                         self.next_state = STATE_PLAN_ERROR
        #                         return
        #             if not flag_found_tag:
        #                 rospy.logerr('Could not find tag to conduct placing task')
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return

        #         except rospy.exceptions.ROSException as e:
        #             rospy.logerr(e)
        #             self.next_state = STATE_PLAN_ERROR
        #             return

        #         p.pose.position.z = FIXED_PLACING_HEIGHT
        #         euler = tf.transformations.euler_from_quaternion([p.pose.orientation.x,
        #                                                             p.pose.orientation.y,
        #                                                             p.pose.orientation.z,
        #                                                             p.pose.orientation.w])
        #         if pi - euler[2] + pi/2 > pi*3/2:
        #             rotation = -euler[2] + pi/2 
        #         elif pi - euler[2] + pi/2 < pi/2:
        #             rotation = pi*2 - euler[2] + pi/2
        #         else:
        #             rotation = pi - euler[2] + pi/2
        #         q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.target_pose = copy.deepcopy(p)
        #         self.pub_target_pose.publish(p)
        #         self.next_state = STATE_PLACING
        #         rospy.loginfo('Ready to place')

        #     elif self.cur_state == STATE_PLACING:
        #         if self.target_pose is not None:
        #             # Go to 5 cm top of placing pose
        #             pre_target_pose = copy.deepcopy(self.target_pose)
        #             pre_target_pose.pose.position.z += 0.05
        #             self.move_group.set_pose_target(pre_target_pose.pose)
        #             self.pub_target_pose.publish(pre_target_pose)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":  # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             rospy.sleep(2)
        #             self.move_group.go(wait=True)
        #             # Go to placing pose
        #             self.move_group.set_pose_target(self.target_pose.pose)
        #             self.pub_target_pose.publish(self.target_pose)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":    # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             rospy.sleep(2)
        #             self.move_group.go(wait=True)
        #             # Release holding object
        #             rospy.wait_for_service('/robotiq_finger_control_node/open_gripper', timeout=3)
        #             open_action = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
        #             open_action()
        #             rospy.sleep(1)
        #             # Go back to 5 cm top of placing pose
        #             post_target_pose = pre_target_pose
        #             self.move_group.set_pose_target(post_target_pose.pose)
        #             self.pub_target_pose.publish(post_target_pose)
        #             plan = self.move_group.plan()
        #             if plan.joint_trajectory.header.frame_id == "":  # No planning result
        #                 self.next_state = STATE_PLAN_ERROR
        #                 return
        #             rospy.sleep(2)
        #             self.move_group.go(wait=True)

        #             if self.object_found_num > 1:
        #                 self.next_state = STATE_POSE_ESTIMATION
        #             else:
        #                 self.next_state = STATE_GO_HOME
        #         else:
        #             rospy.logerr('No target pose to placing')
        #             self.next_state = STATE_PLAN_ERROR
        #             return

        #     elif self.cur_state == STATE_PLAN_ERROR:
        #         rospy.logerr('PLANNING ERROR STATE')

        #     elif self.cur_state == STATE_IMAGE_COLLECTION:
        #         # center
        #         p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #         p.pose.position.x = -0.5
        #         p.pose.position.y = 0.0
        #         p.pose.position.z = 0.4
        #         q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.move_group.set_pose_target(p.pose)
        #         self.pub_target_pose.publish(p)
        #         plan = self.move_group.plan()
        #         rospy.sleep(2)
        #         self.move_group.go(wait=True)

        #         # right
        #         p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #         p.pose.position.x = -0.5
        #         p.pose.position.y = 0.4/sqrt(2)
        #         p.pose.position.z = 0.4/sqrt(2)
        #         q = tf.transformations.quaternion_from_euler(pi/2, pi/4, -pi/2)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.move_group.set_pose_target(p.pose)
        #         self.pub_target_pose.publish(p)
        #         plan = self.move_group.plan()
        #         rospy.sleep(2)
        #         self.move_group.go(wait=True)

        #         # center
        #         p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #         p.pose.position.x = -0.5
        #         p.pose.position.y = 0.0
        #         p.pose.position.z = 0.4
        #         q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.move_group.set_pose_target(p.pose)
        #         self.pub_target_pose.publish(p)
        #         plan = self.move_group.plan()
        #         self.move_group.go(wait=True)

        #         # front
        #         p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #         p.pose.position.x = -0.5 + 0.4/sqrt(2)
        #         p.pose.position.y = 0.0
        #         p.pose.position.z = 0.4/sqrt(2)
        #         q = tf.transformations.quaternion_from_euler(0, pi/4, pi)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.move_group.set_pose_target(p.pose)
        #         self.pub_target_pose.publish(p)
        #         plan = self.move_group.plan()
        #         rospy.sleep(2)
        #         self.move_group.go(wait=True)

        #         # center
        #         p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #         p.pose.position.x = -0.5
        #         p.pose.position.y = 0.0
        #         p.pose.position.z = 0.4
        #         q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.move_group.set_pose_target(p.pose)
        #         self.pub_target_pose.publish(p)
        #         plan = self.move_group.plan()
        #         self.move_group.go(wait=True)

        #         # left
        #         p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
        #         p.pose.position.x = -0.5
        #         p.pose.position.y = -0.4/sqrt(2)
        #         p.pose.position.z = 0.4/sqrt(2)
        #         q = tf.transformations.quaternion_from_euler(-pi/2, pi/4, pi/2)
        #         p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #         self.move_group.set_pose_target(p.pose)
        #         self.pub_target_pose.publish(p)
        #         plan = self.move_group.plan()
        #         rospy.sleep(2)
        #         self.move_group.go(wait=True)
            
    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('system_fsm', anonymous=False)
    node = FSM()
    rospy.on_shutdown(node.shutdown_cb)
    while not rospy.is_shutdown():
        node.process()
        rospy.sleep(0.1)
