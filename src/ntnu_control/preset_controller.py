#!/usr/bin/env python
"""A simple controller"""

import rospy
from sensor_msgs.msg import JointState
import numpy as np


class Controller():
    def __init__(self):
        self.command_rate = rospy.Rate(20)
        self.screw_radius = rospy.get_param("screw_radius")
        self.screw_pitch = rospy.get_param("screw_pitch")
        self.num_joints = rospy.get_param("num_modules")
        self.linear_vel = rospy.get_param("~linear_vel")
        
        self.screw_constant = self.screw_radius * np.sin(self.screw_pitch)
        self.screw_rate = self.linear_vel / self.screw_constant
        
        self.cmd_pub = rospy.Publisher("desired_joint_states", JointState, queue_size=10)
        while (self.cmd_pub.get_num_connections() < 1 and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for subscribers...")
            self.cmd_pub.publish(JointState())
            rospy.sleep(1)
    
    def run(self):
        self.command_path()

    def gen_forward_path(self):
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

            # fun spiral maneuver
            # msg.name.append(f"joint_bend_{i:02d}")
            # msg.velocity.append(0.0)
            # msg.position.append(0.91)
            # msg.effort.append(0.0)
            # msg.name.append(f"joint_twist_{i:02d}")
            # msg.velocity.append(0.0)
            # msg.position.append(1.6)
            # msg.effort.append(0.0)
        return msg
    
    def gen_circular_path(self):
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
        duration = rospy.Duration(5)
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
    
    def test_path(self):
        mgs = JointState()
        duration = rospy.Duration(10)
        msg = self.gen_circular_path()
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(msg)
            self.command_rate.sleep()

            if rospy.Time.now() - start_time > duration:
                break
    


def main():
    rospy.init_node("preset_controller")
    control = Controller()
    control.run()
    rospy.spin()

if __name__ == "__main__":
    main()
