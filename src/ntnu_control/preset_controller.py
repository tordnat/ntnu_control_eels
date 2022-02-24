#!/usr/bin/env python
"""A simple controller"""

import re
import sys
from typing import Dict
from dataclasses import dataclass
import rospy
from sensor_msgs.msg import JointState
import numpy as np


"""
N x M x K
N - antall moduler (1 - 13?)
M - 4 ledd per modul (0: front, 1: rear, 2: bend, 3: twist)
K - 3 variabler per ledd (0: pos, 1: vel, 2: effort)
"""

N_modules = 12
JOINT_TYPES = ['screw_front', 'screw_rear', 'bend', 'twist']

I_SCREW_FRONT = 0
I_SCREW_REAR = 1
I_BEND = 2
I_TWIST = 3

I_POS = 0
I_VEL = 1
I_EFFORT = 2

#Constraints


def decode_ros_msg(msg: JointState) -> np.ndarray:
    arr = np.zeros((N_modules, 4, 3))
    for (joint_name, pos, vel, effort) in zip(msg.name, msg.position, msg.velocity, msg.effort):
        joint_num = int(joint_name.split('_')[-1]) - 1
        joint_type = '_'.join(joint_name.split('_')[1:-1])
        joint_type_i = JOINT_TYPES.index(joint_type)

        arr[joint_num, joint_type_i, :] = (pos, vel, effort)
    return arr
    
def encode_ros_msg(state: np.ndarray) -> JointState:
    print('Array:', state, file=sys.stderr)
    msg = JointState()
    for joint_num in range(0, state.shape[0]):
        for joint_type_i, joint_type in enumerate(JOINT_TYPES):
            pos, vel, effort = state[joint_num, joint_type_i, :]
            msg.name.append(f'joint_{joint_type}_{joint_num+1:02d}')
            msg.position.append(pos)
            msg.velocity.append(vel)
            msg.effort.append(effort)
    print('Message:', msg, file=sys.stderr)
    return msg



###### Moveset #####

def move_stop() -> np.ndarray:
    return np.zeros((N_modules, 4, 3))

def move_forward(vel: float, first: int = 0, last: int = N_modules) -> np.ndarray:
    a = move_stop()
    a[first:last, I_SCREW_FRONT, I_VEL] = vel
    a[first:last, I_SCREW_REAR, I_VEL] = -vel #must be opposite 
    return a

def move_roll_right(vel: float, first: int = 0, last: int = N_modules) -> np.ndarray:
    a = move_stop()
    a[first:last, I_SCREW_FRONT, I_VEL] = vel
    a[first:last, I_SCREW_REAR, I_VEL] = vel
    return a

def move_bend(angle: float, first: int = 0, last: int = N_modules) -> np.ndarray:
    a = move_stop()
    a[first:last, I_BEND, I_POS] = -angle
    return a

def move_twist(angle: float, first: int = 0, last: int = N_modules) -> np.ndarray:
    a = move_stop()
    a[first:last, I_TWIST, I_POS] = angle
    return a

def move_incline(angle: float, module: int) -> np.ndarray:
    a = move_stop()
    a += move_bend(angle, module, module+1)
    a += move_twist(np.pi / 2, module, module+1)
    return a


class Controller():
    _actual_state: np.ndarray

    def __init__(self):
        self._actual_state = move_stop()
        self.command_rate = rospy.Rate(20)
        self.screw_radius = rospy.get_param("screw_radius")
        self.screw_pitch = rospy.get_param("screw_pitch")
        self.num_joints = rospy.get_param("num_modules")
        self.linear_vel = rospy.get_param("~linear_vel")
        
        self.screw_constant = self.screw_radius * np.sin(self.screw_pitch)
        self.screw_rate = self.linear_vel / self.screw_constant

        self.state_sub = rospy.Subscriber("joint_states", JointState, callback=self._on_recv_state)

        
        self.cmd_pub = rospy.Publisher("desired_joint_states", JointState, queue_size=10)
        while (self.cmd_pub.get_num_connections() < 1 and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for subscribers...")
            self.cmd_pub.publish(JointState())
            rospy.sleep(1)
    

    def _on_recv_state(self, msg: JointState):
        """
        - joint_screw_front_{num}
        - joint_screw_rear_{num}
        - joint_bend_{num}
        - joint_twist_{num}
        """
        self._actual_state = decode_ros_msg(msg)


    def run(self):
        self.command_path()

    def gen_forward_path(self):
        rospy.loginfo("Generating forward path")
        
        return encode_ros_msg(
            # + move_forward(10)
            # + move_bend(np.pi / 2, 11, 12)
            + move_incline(np.pi / 4, 11)
        )


    def gen_bends(self, pos: int):
        rospy.loginfo("Generating rise")
        msg = JointState()
        for i in range(1, self.num_joints + 1, 1):
            msg.name.append("joint_screw_front_{i:02d}")
            msg.name.append("joint_screw_back_{i:02d}")
            msg.name.append("joint_bend_{i:02d}")
        return msg
    
    def gen_circular_path(self):
        rospy.loginfo("Generating circular path")
        msg = JointState()
        for i in range(1, self.num_joints + 1, 1):
            msg.name.append(f"joint_screw_front_{i:02d}")
            msg.velocity.append(self.screw_rate)
            msg.position.append(0.0)
            msg.effort.append(0.0)

            msg.name.append(f"joint_screw_rear_{i:02d}")
            msg.velocity.append(-self.screw_rate)
            msg.position.append(0.0)
            msg.effort.append(0.0)

            msg.name.append(f"joint_bend_{i:02d}")
            msg.velocity.append(0.0)
            msg.position.append(0.4)
            msg.effort.append(0.0)        
        return msg
    
    def gen_stop(self):
        rospy.loginfo("Generating stop path")
        msg = JointState()
        for i in range(1, self.num_joints + 1, 1):
            msg.name.append(f"joint_screw_front_{i:02d}")
            msg.velocity.append(0.0)
            msg.position.append(0.0)
            msg.effort.append(0.0)

            msg.name.append(f"joint_screw_rear_{i:02d}")
            msg.velocity.append(-0.0)
            msg.position.append(0.0)
            msg.effort.append(0.0)     
        return msg
    
    def command_path(self):
        # Send forward velocity for the first 5 seconds
        rospy.loginfo("Start forward motion")
        duration = rospy.Duration(60)
        msg = self.gen_forward_path()
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(msg)
            self.command_rate.sleep()

            if rospy.Time.now() - start_time > duration:
                break

        # Send rotating in the next 15 seconds
        rospy.loginfo("Start circular motion")
        duration = rospy.Duration(15)
        msg = self.gen_circular_path()
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(msg)
            self.command_rate.sleep()
            if rospy.Time.now() - start_time > duration:
                break
        
        rospy.loginfo("Stopping all commanded motion")
        msg = self.gen_stop()
        self.cmd_pub.publish(msg)
    


def main():
    rospy.init_node("preset_controller")
    control = Controller()
    control.run()
    rospy.spin()

if __name__ == "__main__":
    main()
