#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This module provides a class that controls the serial servo motor manufactured by Futaba Corp.
# ver1.37622
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# (C) 2015 Hiroaki Matsuda and Naohiro Hayashi
# 2016 Asano 
import math
import pymx
import rospy
import tf
import sys
from std_msgs.msg import String

class Hook(object):
        def __init__(self, port , baudrate, timeout = 1):
                self.mx = pymx.Mx()
                self.mx.open_port(port, baudrate, timeout)
                self._sub_hook = rospy.Subscriber('hook', String, self.callback)
                self.br = tf.TransformBroadcaster() 

        def tf_left_broadcast(self, theta):
                R = tf.transformations.rotation_matrix(theta / 2 * math.pi / 180, (1, 0, 0))
                q = tf.transformations.quaternion_from_matrix(R)
                self.br.sendTransform((0, -0.120 * math.sin(theta / 2 * math.pi / 180), 0.120 * math.cos(theta * 0.5 * math.pi / 180 )), q, rospy.Time.now(), "left_endgripper3", "left_gripper")
                print("tf_left_broadcast!")

        def tf_right_broadcast(self, theta):
                theta = -theta
                R = tf.transformations.rotation_matrix(theta / 2 * math.pi / 180, (1, 0, 0))
                q = tf.transformations.quaternion_from_matrix(R)
                self.br.sendTransform((0, -0.115 * math.sin(theta / 2 * math.pi / 180 ), 0.115 * math.cos(theta * 0.5 * math.pi / 180)), q, rospy.Time.now(), "right_endgripper3", "right_gripper")
                print("tf_right_broadcast!")
                
        def initialize_gripper(self, speed = 20):
                self.mx.speed(1, speed)
                self.mx.speed(2, speed)

                self.servo_gripper(1, 'all')
                self.move_gripper(37, 'all')

        def finalize_gripper(self):
                print("clean gripper!")
                #~ self.servo_gripper(180, 'all')
                self.mx.close_port()
                return True

        def servo_gripper(self, enable = 0, gripper = 'all'):
                if gripper is 'left':
                        self.mx.reg_servo([1],
                                          [enable])
                        self.mx.action(254)

                elif gripper is 'right':
                        self.mx.reg_servo([2],
                                          [enable])
                        self.mx.action(254)

                elif gripper is 'all':
                        self.mx.reg_servo([1, 2],
                                          [enable, enable])
                        self.mx.action(254)

        def open_gripper(self, width):
                self.mx.reg_move([1, 2], [width, width])
                self.mx.action(254)

        def move_gripper(self, width = 0, gripper = 'all'):
                if gripper is 'left':
                        self.mx.reg_move([1],
                                         [180 + width])
                        self.mx.action(254)
                        self.tf_left_broadcast(width)

                elif gripper is 'right':
                        self.mx.reg_move([2],
                                         [180 - width])
                        self.mx.action(254)
                        self.tf_right_broadcast(width)

                elif gripper is 'all':
                        self.mx.reg_move([1, 2],
                                         [180 + width,
                                          180 - width])
                        self.mx.action(254)
                        self.tf_right_broadcast(width)
                        self.tf_left_broadcast(width)

        def callback(self, cmd_string):
                if cmd_string.data is 'a':
                        self.move_gripper(37, 'left')

                elif cmd_string.data is 'b':
                        self.move_gripper(0, 'left')

                elif cmd_string.data is 'c':
                        self.move_gripper(37, 'right')

                elif cmd_string.data is 'd':
                        self.move_gripper(0, 'right')

                elif cmd_string.data is 'e':
                        self.move_gripper(37, 'all')

                elif cmd_string.data is 'f':
                        self.move_gripper(0, 'all')
                        
                elif cmd_string.data is 'g':
                        self.move_gripper(0, 'left')
                        self.move_gripper(37, 'right')

                elif cmd_string.data is 'h':
                        self.move_gripper(37, 'left')
                        self.move_gripper(0, 'right')
                        
                elif cmd_string.data[0] is 'r':
                    cmd_float = float(cmd_string.data[2:])
                    if cmd_float >= 0 and cmd_float <= 60.0:
                        self.move_gripper(cmd_float, 'right')
                    else:
                        rospy.loginfo(rospy.get_name() + ": hook_node heard %s. Its msg is false! " % cmd_string.data)
            
                elif cmd_string.data[0] is 'l':
                    cmd_float = float(cmd_string.data[2:])
                    if cmd_float >= 0 and cmd_float <= 60.0:
                        self.move_gripper(cmd_float, 'left')
                    else:
                        rospy.loginfo(rospy.get_name() + ": hook_node heard %s. Its msg is false!" % cmd_string.data)           
                else:
                    rospy.loginfo(rospy.get_name() + ": hook_node heard %s. Its msg is false!" % cmd_string.data)

def main():
    import time

    rospy.init_node('Hook_node', anonymous=False)
    hook = Hook('/dev/ttyUSB0', 3000000, 1)
    print("initialize gripper")
    hook.initialize_gripper(5)
    print("wait command")
    rospy.spin()
    rospy.on_shutdown(hook.finalize_gripper())


if __name__ == '__main__':
    sys.exit(main())
