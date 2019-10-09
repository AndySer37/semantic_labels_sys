#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from text_msgs.srv import *
from text_msgs.msg import *

Detect_SRV = "/text_detection/text_detection"
Pose_SRV = "/depth_to_pose/get_pose"
mani_req = manipulationRequest()

class Perception(smach.State):
    def __init__(self, per_client, pose_client):
        smach.State.__init__(
            self,
            outcomes=['finish'])
        self.per_client = per_client
        self.pose_client = pose_client

    def execute(self, userdata):
        global mani_req
        mani_req =  manipulationRequest()

        per_res = self.per_client() #Call service
        pose_req = pose_stateRequest()
        pose_req.image = per_res.image
        pose_req.depth = per_res.depth 
        pose_req.mask = per_res.mask
        pose_res = self.pose_client(pose_req)

        # mani_req.brandname = res.brand
        # mani_req.target = res.pose
        # if res.result_type == 0:
        #     mani_req.mode = 0
        #     rospy.loginfo("Brand name is visible")
        #     return 'v'
        # elif res.result_type == 1:
        #     mani_req.mode = 2
        #     rospy.loginfo("Object is visible")
        #     return 'nv'
        # else:
        print pose_res.count 
        rospy.loginfo("No object. Finish the task")
        return "finish"



# class Perception_gripper(smach.State):
#     def __init__(self, per_client):
#         smach.State.__init__(
#             self,
#             outcomes=['v', 'nv', 'np'])
#         self.per_client = per_client
#         self.try_num = 0
        
#     def execute(self, userdata):
#         global mani_req
#         res = self.per_client(2)
#         mani_req =  manipulationRequest()
#         mani_req.brandname = res.brand
#         mani_req.target = res.pose
#         if res.result_type == 0:
#             rospy.loginfo("Brand name is visible")
#             mani_req.mode = 6
#             return "v"
#         elif res.result_type == 1:
#             rospy.loginfo("Brand name is invisible")
#             if self.try_num == 0:
#                 mani_req.mode = 4
#                 self.try_num += 1
#                 return "nv"
#             else:
#                 mani_req.mode = 5
#                 self.try_num = 0
#                 rospy.loginfo("Fail this trial")
#                 return 'np'
#         else:
#             mani_req.mode = 5
#             self.try_num = 0
#             rospy.loginfo("Fail this trial")
#             return 'np'

# class Pick_and_place(smach.State):
#     def __init__(self, act_client):
#         smach.State.__init__(
#             self,
#             outcomes=['finish']
#             )
#         self.act_client = act_client
        
#     def execute(self, userdata):
#         global mani_req
#         self.act_client(mani_req) 
#         return 'finish'


# class Pick_to_uncluttered_tote(smach.State):
#     def __init__(self, act_client):
#         smach.State.__init__(
#             self,
#             outcomes=['finish'])
#         self.act_client = act_client
        
#     def execute(self, userdata):
#         global mani_req
#         self.act_client(mani_req)
#         return 'finish'


# class Pick(smach.State):
#     def __init__(self, act_client):
#         smach.State.__init__(
#             self,
#             outcomes=['finish'])
#         self.act_client = act_client
        
#     def execute(self, userdata):
#         global mani_req
#         self.act_client(mani_req)
#         return 'finish'

# class Change_object_pose(smach.State):
#     def __init__(self, act_client):
#         smach.State.__init__(
#             self,
#             outcomes=['finish'])
#         self.act_client = act_client
        
#     def execute(self, userdata):
#         global mani_req
#         self.act_client(mani_req)
#         return 'finish'


# class Put_back_to_tote(smach.State):
#     def __init__(self, act_client):
#         smach.State.__init__(
#             self,
#             outcomes=['finish'])
#         self.act_client = act_client
#         global mani_req
#     def execute(self, userdata):
#         self.act_client(mani_req)
#         return 'finish'



def main():
    rospy.init_node('Semantic_System')
    rospy.wait_for_service(Detect_SRV)
    rospy.wait_for_service(Pose_SRV)

    print "Server Ready !!!"
    perception_client = rospy.ServiceProxy(Detect_SRV, text_detection_srv)
    pose_client = rospy.ServiceProxy(Pose_SRV, pose_state)
    # action_client = rospy.ServiceProxy('manipulation_server', manipulation)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Finish'])
    
    with sm:
        # Add Perception_tote state to the container
        smach.StateMachine.add('Perception', Perception(perception_client, pose_client),
            transitions={'finish':'Finish'})
            
        # # Add Perception_platform state to the container    
        # smach.StateMachine.add('Perception_platform', Perception_platform(perception_client),
        #     transitions={'v':'Pick_and_place', 'nv':'Perception_tote'})
        # # # Add Perception_gripper state to the container         
        # smach.StateMachine.add('Perception_gripper', Perception_gripper(perception_client),
        #     transitions={'v':'Pick_and_place',
        #     'nv':'Change_object_pose', 'np':'Put_back_to_tote'})


        # # Add Pick_to_uncluttered_tote state to the container
        # smach.StateMachine.add('Pick_to_uncluttered_tote', Pick_to_uncluttered_tote(action_client),
        #     transitions={'finish':'Perception_platform'})#Perception_platform


        # # Add Pick_and_place state to the container
        # smach.StateMachine.add('Pick_and_place', Pick_and_place(action_client),
        #     transitions={'finish':'Perception_tote'})

        # # Add Pick_and_check2 state to the container
        # smach.StateMachine.add('Pick', Pick(action_client),
        #     transitions={'finish':'Perception_gripper'})
        # # Add Change_object_pose state to the container
        # smach.StateMachine.add('Change_object_pose', Change_object_pose(action_client),
        #     transitions={'finish':'Perception_gripper'})
 
        # # Add Put_back_to_tote state to the container
        # smach.StateMachine.add('Put_back_to_tote', Put_back_to_tote(action_client),
        #     transitions={'finish':'Perception_tote'})

    semantic_system = smach_ros.IntrospectionServer('Active_manipulation', sm,
        '/Active_manipulation')
    semantic_system.start()
    outcome = sm.execute()
    rospy.spin()
    semantic_system.stop()

if __name__ == '__main__':
    main()
