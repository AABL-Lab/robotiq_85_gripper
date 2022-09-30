"""--------------------------------------------------------------------
COPYRIGHT 2015 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company")
for its licensed Segway RMP Robotic Platforms is intended and supplied to you,
the Company's customer, for use solely and exclusively with Stanley Innovation
products. The software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws.  All rights are reserved. Any use in
violation of the foregoing restrictions may subject the user to criminal
sanctions under applicable laws, as well as to civil liability for the
breach of the terms and conditions of this license. The Company may
immediately terminate this Agreement upon your use of the software with
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use
of the software is therefore subject to the terms and conditions of the
OSI- approved open source license viewable at http://www.python.org/.
You are solely responsible for ensuring your compliance with the Python
open source license.

You shall indemnify, defend and hold the Company harmless from any claims,
demands, liabilities or expenses, including reasonable attorneys fees, incurred
by the Company as a result of any claim or proceeding against the Company
arising out of or based upon:

(i) The combination, operation or use of the software by you with any hardware,
    products, programs or data not supplied or approved in writing by the Company,
    if such claim or proceeding would have been avoided but for such combination,
    operation or use.

(ii) The modification of the software by or on behalf of you

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 \file   robotiq_85_driver.py

 \brief  Driver for Robotiq 85 communication

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import time
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from robotiq_85_msgs.msg import GripperCmd, GripperStat

from .robotiq_85_gripper import Robotiq85Gripper

class Robotiq85Driver:
    def __init__(self):
        self._num_grippers = rospy.get_param('~num_grippers',1)
        self._comport = rospy.get_param('~comport','/dev/ttyUSB0')
        self._baud = rospy.get_param('~baud','115200')

        connected = False
        printed = False
        while not connected and not rospy.is_shutdown():
            self._gripper = Robotiq85Gripper(self._num_grippers,self._comport,self._baud)
            connected = self._gripper.init_success
            if not connected:
                if not printed:
                    rospy.logerr("Unable to open comport to %s while connecting to Robotiq 85 gripper. Will keep trying..." % self._comport)
                    printed = True
                rospy.sleep(1.0)
        if rospy.is_shutdown():
            print("ROS shutdown while connecting to Robotiq 85 gripper")
            return

        if (self._num_grippers == 1):
            rospy.Subscriber("/gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
            self._gripper_pub = rospy.Publisher('/gripper/stat', GripperStat, queue_size=10)
            self._gripper_joint_state_pub = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)
        elif (self._num_grippers == 2):
            rospy.Subscriber("/left_gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
            self._left_gripper_pub = rospy.Publisher('/left_gripper/stat', GripperStat, queue_size=10)
            self._left_gripper_joint_state_pub = rospy.Publisher('/left_gripper/joint_states', JointState, queue_size=10)
            rospy.Subscriber("/right_gripper/cmd", GripperCmd, self._update_right_gripper_cmd, queue_size=10)
            self._right_gripper_pub = rospy.Publisher('/right_gripper/stat', GripperStat, queue_size=10)
            self._right_gripper_joint_state_pub = rospy.Publisher('/right_gripper/joint_states', JointState, queue_size=10)
        else:
            rospy.logerr("Number of grippers not supported (needs to be 1 or 2)")
            return

        self._seq = [0] * self._num_grippers
        self._prev_js_pos = [0.0] * self._num_grippers
        self._prev_js_time = [rospy.get_time()] * self._num_grippers
        self._driver_state = 0
        self._driver_ready = False

        connected = False
        printed = False
        while not connected and not rospy.is_shutdown():
            connected = True
            for i in range(self._num_grippers):
                connected &= self._gripper.process_cmds(i)
                if not connected and not printed:
                    rospy.logerr("Failed to contact gripper %d. Will keep trying..."%i)
            if not connected:
                printed = True
                rospy.sleep(1.0)
        if rospy.is_shutdown():
            print("ROS shutdown while connecting to Robotiq 85 gripper")
            return

        rospy.loginfo("Robotiq 85 driver(s) connected successfully.")
        self._run_driver()

    def _clamp_cmd(self,cmd,lower,upper):
        if (cmd < lower):
            return lower
        elif (cmd > upper):
            return upper
        else:
            return cmd

    def _update_gripper_cmd(self,cmd,dev=0):
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop(dev=dev)
        else:
            pos = self._clamp_cmd(cmd.position,0.0,1.0)
            vel = self._clamp_cmd(cmd.speed,0.0, 255.0)
            force = self._clamp_cmd(cmd.force,0.0,225.0)
            self._gripper.goto(dev=dev,pos=pos,vel=vel,force=force)

    def _update_right_gripper_cmd(self,cmd):
        self._update_gripper_cmd(dev=1)

    def _create_gripper_stat_msg(self, dev):
        """
        create a GripperState ROS message based on current status of gripper
        """

        stat = GripperStat()
        stat.header.stamp = rospy.get_rostime()
        stat.header.seq = self._seq[dev]
        stat.is_ready = self._gripper.is_ready(dev)
        stat.is_reset = self._gripper.is_reset(dev)
        stat.is_moving = self._gripper.is_moving(dev)
        stat.obj_detected = self._gripper.object_detected(dev)
        stat.fault_status = self._gripper.get_fault_status(dev)
        stat.position = self._gripper.get_pos(dev)
        stat.requested_position = self._gripper.get_req_pos(dev)
        stat.current = self._gripper.get_current(dev)
        self._seq[dev]+=1
        return stat

    def _create_joint_state_msg(self, dev):
        """
        Create a JointState message based on the current status of the gripper
        """
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = self._seq[dev]
        js.name = ['gripper_finger1_joint']
        pos = self._gripper.get_pos(dev)
        js.position = [pos]
        dt = rospy.get_time() - self._prev_js_time[dev]
        self._prev_js_time[dev] = rospy.get_time()
        js.velocity = [(pos-self._prev_js_pos[dev])/dt]
        self._prev_js_pos[dev] = pos
        return js

    def _run_driver(self):
        last_time = rospy.get_time()
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            dt = rospy.get_time() - last_time
            if (0 == self._driver_state):
                for i in range(self._num_grippers):
                    if (dt < 0.5):
                        self._gripper.deactivate_gripper(i)
                    else:
                        self._driver_state = 1
            elif (1 == self._driver_state):
                grippers_activated = True
                for i in range(self._num_grippers):
                    self._gripper.activate_gripper(i)
                    grippers_activated &= self._gripper.is_ready(i)
                if (grippers_activated):
                    self._driver_state = 2
            elif (2 == self._driver_state):
                self._driver_ready = True

            for i in range(self._num_grippers):
                try:
                    self._gripper.process_cmds(i)
                except Exception as e:
                    if rospy.is_shutdown():
                        rospy.loginfo("Shutting down Robotiq driver")
                    else:
                        rospy.logerr("Robotiq error type: " + str(type(e)))
                        rospy.logerr("Robotiq communication error: " + str(e))

                stat = self._create_gripper_stat_msg(i)
                js = self._create_joint_state_msg(i)
                if (1 == self._num_grippers):
                    self._gripper_pub.publish(stat)
                    self._gripper_joint_state_pub.publish(js)
                else:
                    if (i == 0):
                        self._left_gripper_pub.publish(stat)
                        self._left_gripper_joint_state_pub.publish(js)
                    else:
                        self._right_gripper_pub.publish(stat)
                        self._right_gripper_joint_state_pub.publish(js)

            r.sleep()

        self._gripper.shutdown()
