#!/usr/bin/env python
"""A simple controller"""

import re
import sys
from typing import Dict, Tuple
from dataclasses import dataclass
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, QuaternionStamped, Vector3Stamped
import numpy as np
import tf
import tf.transformations

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
    msg = JointState()
    for joint_num in range(0, state.shape[0]):
        for joint_type_i, joint_type in enumerate(JOINT_TYPES):
            pos, vel, effort = state[joint_num, joint_type_i, :]
            msg.name.append(f'joint_{joint_type}_{joint_num+1:02d}')
            msg.position.append(pos)
            msg.velocity.append(vel)
            msg.effort.append(effort)
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
    a[first:last, I_BEND, I_POS] = -angle / (last - first)
    return a

def move_twist(angle: float, first: int = 0, last: int = N_modules) -> np.ndarray:
    a = move_stop()
    a[first:last, I_TWIST, I_POS] = angle / (last - first)
    return a

def move_incline(angle: float, module: int) -> np.ndarray:
    a = move_stop()
    a += move_twist(np.pi / 2, module-1, module)
    a += move_bend(-angle, module, module+1)
    return a

def move_roll_proportional(vel: float) -> np.ndarray:
    a = move_stop()
    for module in range(N_modules):
        a += move_roll_right(vel, module, module+1) * (module / N_modules)
    return a

def make_line_config() -> np.ndarray:
    return move_bend(0)

def make_u_config() -> np.ndarray:
    return move_bend(np.pi / 2)

def make_screw_vectors(current_global_angles: np.ndarray, target_direction: Tuple[float, float, float]) -> np.ndarray:
    """Make screw forces that brings us in a target direction"""
    # global heading angle
    target_global_angle = np.arctan2(target_direction[1], target_direction[0])
    # relative heading angle for each module
    module_force_angles = current_global_angles - target_global_angle

    """
    Velocity is a weighted sum of the forward and right unit vectors:
        v(q1, q2) = [0 1](q1 - q2) + [1 0](q1 + q2)
            = [
                q1 + q2
                q1 - q2
            ]
    Velocity is also determined by the relative target heading phi:
        [ cos(phi), sin(phi) ] = [ (q1+q2), (q1-q2) ]
    Solving for q1 and q2:
        q1 + q2 = cos(phi)
        q1 - q2 = sin(phi)
        2q1 = cos(phi) + sin(phi)
        2q2 = cos(phi) - sin(phi)
    """
    
    scale = 0.2
    print(scale)
    
    res = move_stop()
    res[:, I_SCREW_FRONT, I_VEL] = scale * (np.cos(module_force_angles) + np.sin(module_force_angles))
    res[:, I_SCREW_REAR, I_VEL] = scale * (np.cos(module_force_angles) - np.sin(module_force_angles))
    return res

def make_roll_help(configuration_error: np.ndarray) -> np.ndarray:
    relative_bends = configuration_error[:, I_BEND, I_POS]
    absolute_bends = np.cumsum(relative_bends)
    absolute_bends -= np.mean(absolute_bends)

    c = 0.2

    roll_helps = move_stop()
    roll_helps[:, I_SCREW_FRONT, I_VEL] = -c * absolute_bends
    roll_helps[:, I_SCREW_REAR, I_VEL] = -c * absolute_bends
    return roll_helps

def orientation_to_global_angle(orientation):
    _, _, yaw = tf.transformations.euler_from_quaternion([
        orientation.quaternion.x,
        orientation.quaternion.y,
        orientation.quaternion.z,
        orientation.quaternion.w
    ])
    return yaw

def orientations_to_global_angles(orientations):
    return list(map(orientation_to_global_angle, orientations))

class Controller():
    _actual_state: np.ndarray

    def __init__(self):
        self._actual_state = None
        self.command_rate = rospy.Rate(100)
        self.screw_radius = rospy.get_param("screw_radius")
        self.screw_pitch = rospy.get_param("screw_pitch")
        self.num_joints = rospy.get_param("num_modules")
        self.linear_vel = rospy.get_param("~linear_vel")
        
        self.screw_constant = self.screw_radius * np.sin(self.screw_pitch)
        self.screw_rate = self.linear_vel / self.screw_constant

        self.state_sub = rospy.Subscriber("joint_states", JointState, callback=self._on_recv_state)
        # self.odom_sub = rospy.Subscriber("odometry", Odometry, callback=self._on_recv_odom)

        self.listener = tf.TransformListener()
        self.link_orientations = []
        self.global_angles = []
        
        self.cmd_pub = rospy.Publisher("desired_joint_states", JointState, queue_size=10)
        while (self.cmd_pub.get_num_connections() < 1 and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for subscribers...")
            self.cmd_pub.publish(JointState())
            rospy.sleep(1)
    
    def _on_recv_odom(self, msg):
        pass

    def _on_recv_state(self, msg: JointState):
        """
        - joint_screw_front_{num}
        - joint_screw_rear_{num}
        - joint_bend_{num}
        - joint_twist_{num}
        """
        self._actual_state = decode_ros_msg(msg)
        print(self._actual_state[:, I_SCREW_FRONT, I_VEL])


    def run(self):
        self.command_path()

    def gen_path(self):
        self.link_orientations = self.get_link_orientations()
        self.global_angles = orientations_to_global_angles(self.link_orientations)
        configuration_target = make_u_config()
        if self._actual_state is None:
            return configuration_target
        else:
            now = rospy.get_time()
            translation_vector = (-1, 0, 0) # (np.cos(0.1*now), np.sin(0.1*now), 0)
            # translation_movement = make_screw_vectors(np.array(self.global_angles), (0, 1, 0))
            translation_movement = make_screw_vectors(np.array(self.global_angles), translation_vector)
            configuration_error = self._actual_state - configuration_target
            return translation_movement \
                    + configuration_target \
                    + make_roll_help(configuration_error) \
                    

    def gen_forward_path(self):
        rospy.loginfo("Generating forward path")
        
        return encode_ros_msg(
            # + move_forward(10)
            # + move_twist(np.pi / 2, 9, 10)
            # + move_bend(-np.pi / 2)
            # + move_roll_proportional(1)
            move_incline(np.pi / 4, 8)
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

        while not rospy.is_shutdown():
            msg = encode_ros_msg(self.gen_path())
            msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(msg)
            self.command_rate.sleep()

    def get_link_orientations(self):
        link_orientations = []
        global_angles = []

        now = rospy.Time.now()

        for i in range(1, self.num_joints + 1, 1):
            unit_quat = QuaternionStamped()
            unit_quat.header.stamp = now
            unit_quat.header.frame_id = f"link_body_front_{i:02d}"
            self.listener.waitForTransform(
                f"link_body_front_{i:02d}", "map",
                unit_quat.header.stamp, rospy.Duration(1.0))
            link_orientation = self.listener.transformQuaternion("map", unit_quat)
            link_orientations.append(link_orientation)

        return link_orientations


def main():
    rospy.init_node("preset_controller")
    control = Controller()
    control.run()
    rospy.spin()

if __name__ == "__main__":
    main()
