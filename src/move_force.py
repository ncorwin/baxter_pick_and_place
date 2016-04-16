#!/usr/bin/env python

import rospy
import baxter_interface
from copy import copy
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose, Wrench
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from quat import quat_to_so3
import numpy as np
from math import fabs
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
from functions import JointTrajectory, ScrewTrajectory, CartesianTrajectory

from baxter_core_msgs.msg import EndpointState

from collections import deque

import jamming_controll as jam

class Trajectory(object):
    def __init__(self, limb):
        self._done = False
        self._state = 0
        self._Farray = deque([], maxlen = 50)
        
    def set_pos_callback(self, data):
        self._euclidean_goal = data

#        if self._state == 2:
#            self.execute_move(data)

        if self._state == 4:
            self.fine_move(data)
    
    def set_state_callback(self, data):
        self._state = data.data

    def force_callback(self, data):
        #print data
        self._force = data

        self._Farray.append(self._force.wrench.force.z)

        self._Favg = np.mean(self._Farray)
    
    def execute_move(self, pos, raw=False):
        #rospy.loginfo('moving')
        # Read in pose data
        q = [pos.orientation.w, pos.orientation.x, pos.orientation.y, pos.orientation.z]
        p =[[pos.position.x],[pos.position.y],[pos.position.z]]
        # Convert quaternion data to rotation matrix
        R = quat_to_so3(q);
        # Form transformation matrix
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        # Create seed with current position
        q0 = kdl_kin.random_joint_angles()
        limb_interface = baxter_interface.limb.Limb('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]
        pose = kdl_kin.forward(q0)
        Xstart = copy(np.asarray(pose))
        pose[0:3,0:3] = R
        pose[0:3,3] = p
        Xend = copy(np.asarray(pose))
        
        # Compute straight-line trajectory for path
        N = 500
        Xlist = CartesianTrajectory(Xstart, Xend, 1, N, 5)
        thList = np.empty((N,7))
        thList[0] = q0;
        
        for i in range(N-1):
            # Solve for joint angles
            seed = 0
            q_ik = kdl_kin.inverse(Xlist[i+1], thList[i])
            while q_ik == None:
                seed += 0.3
                q_ik = kdl_kin.inverse(pose, q0+seed)
            thList[i+1] = q_ik
#            rospy.loginfo(q_ik)
        
#        # Solve for joint angles
#        seed = 0.3
#        q_ik = kdl_kin.inverse(pose, q0+seed)
#        while q_ik == None:
#            seed += 0.3
#            q_ik = kdl_kin.inverse(pose, q0+seed)
#        rospy.loginfo(q_ik)
        
#        q_list = JointTrajectory(q0,q_ik,1,100,5)
        
#        for q in q_list:
            # Format joint angles as limb joint angle assignment      
            angles = limb_interface.joint_angles()
            angle_count = 0
            for ind, joint in enumerate(limb_interface.joint_names()):
#                if fabs(angles[joint] - q_ik[ind]) < .05:
#                    angle_count += 1
                angles[joint] = q_ik[ind]
#            rospy.loginfo(angles)
            rospy.sleep(.003)
            
            # Send joint move command
            #rospy.loginfo('move to object')
#            rospy.loginfo(angle_count)
#            if angle_count > 4:
#                nothing = True;
#            else:
            limb_interface.set_joint_position_speed(.3)
            limb_interface.set_joint_positions(angles, raw)
        self._done = True
        print('Done')

    def fine_move(self, xstart):
        #set initial position
        print("start")

        jam.Reset(0.1)

        xmod = xstart
        Fmin = -4

        #F = deque([], maxlen = 5)
        #zforce = self._force.wrench.force.z

        #print self._force.wrench.force.z

        #F.append(self._force.wrench.force.z)

        #test = np.mean(F)
                
        while self._force.wrench.force.z > Fmin: #loop checking to see if e-e is close enough to table
            print("while")
            print("Z force mean", self._Favg)
            print("Z force", self._force.wrench.force.z)

            jam.Reset(0.01)

            #keep all posiiton and orientation of e-e except for height the same for each itteration of loop
            xmod.position.x = xstart.position.x
            xmod.position.y = xstart.position.y

            xmod.orientation.x = xstart.orientation.x
            xmod.orientation.y = xstart.orientation.y
            xmod.orientation.z = xstart.orientation.z
            xmod.orientation.w = xstart.orientation.w

            #rospy.loginfo('moving down')
            xmod.position.z = xmod.position.z - 0.05

            
            rospy.loginfo(xmod.position.z)
#            rospy.loginfo(limb_interface.endpoint_pose())
            self.execute_move(xmod) #Carteasian straight line trajectory to modified (lower z) e-e position
            

        #Changes the state to 5 in the state machine when e-e is close enough to pick up block

        print("Z force mean", self._Favg)
        print("Z force", self._force.wrench.force.z)
        #jam.Reset(0.05)

        xmod.position.x = xstart.position.x
        xmod.position.y = xstart.position.y

        xmod.orientation.x = xstart.orientation.x
        xmod.orientation.y = xstart.orientation.y
        xmod.orientation.z = xstart.orientation.z
        xmod.orientation.w = xstart.orientation.w

        #rospy.loginfo('moving down')
        xmod.position.z = xmod.position.z - .01

        self.execute_move(xmod, True)
            
        rospy.loginfo(xmod.position.z)
        #self.execute_move(xmod)

        print("out")
        
        jam.Grip(1)

        rospy.sleep(1)
        
        pub_state = rospy.Publisher('state', Int16, queue_size = 10, latch=True)
        rospy.loginfo(5) 
        rospy.sleep(0.2)
        pub_state.publish(5)          
                          
        self._done = True
        print('Done')
        
        
def main():
    jam.Grip()

    rospy.init_node('move_force')
    traj = Trajectory('right')
    rospy.Subscriber("hand_position", Pose, traj.set_pos_callback)
    rospy.Subscriber("state", Int16, traj.set_state_callback)
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, traj.force_callback)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
