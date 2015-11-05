#!/usr/bin/env python

"""
This is a module for reading 9DOF razor IMU data
"""
import serial
import time
import binascii
import struct
import threading
#import getch
import numpy as np

import roslib; roslib.load_manifest('razor_imu_pkg')
from razor_imu_pkg.srv import *
import rospy
from sensor_msgs.msg import Imu
import tf.transformations as tftrans


dev_port = '/dev/ttyUSB0'
baud_rate = 57600
syn_token = '#SYNCH00\r\n'

yaw = 0.0
pitch = 0.0
roll = 0.0
yawOffset = 0.0

acc_x = 0.0
acc_y = 0.0
acc_z = 0.0

topic = '/imu_node/data'
service = '/imu_node/cmd'

bSync = False
bZero = False

#zero reading for accels
num_zero_time = 100 #2 secs
zero_timer_cnt = 0
#lock = threading.Lock()

# """
# thread for detecting key press event
# """
# this seems not very responsive, probably the main stream is occupying the keyboard reading...
# class KeyEventThread(threading.Thread):
#   def run(self):
#       while True:
#           #key = getch.getch()
#           key = raw_input()
#           if ord(key) == 'a' or ord(key) == 'A':
#               """
#               do alignment
#               """
#               with lock:
#                   yawOffset = yaw
#               rospy.loginfo('Alignment with current yaw angle...')


def read_float(serial):
    """
    helper function to read a float from the serial port
    """
    #convert from little endian(Razor) to big endian
    hex_array = reversed([chr(ord(serial.read(1))) for i in range(4)])
    hex_string = ''.join(hex_array)
    return struct.unpack('>f', hex_string)[0]

def imu_cmd_handler(req):
    resp = None
    #check command
    if req.cmd == 'align':
        global bZero, yawOffset, yaw, zero_timer_cnt
        #do align here...
        yawOffset = yaw
        bZero = False
        zero_timer_cnt = 0
        rospy.loginfo('RAZOR_IMU_NODE: Aligning yaw to current pose...')
        resp = 'Yaw direction aligned'
    else:
        rospy.logerr('RAZOR_IMU_NODE: Invalid requested service command: %s', req.cmd)
        resp = 'Invalid command'
    return resp

def main():
    global yaw, pitch, roll, yawOffset
    global acc_x, acc_y, acc_z
    global bSync, bZero, zero_timer_cnt, num_zero_time

    ser = serial.Serial(dev_port, baud_rate)

    tmp_acc_x = 0.0
    tmp_acc_y = 0.0
    tmp_acc_z = 0.0

    acc_x_zero = 0.0
    acc_y_zero = 0.0
    acc_z_zero = 0.0

    #scale for one gravity
    acc_x_scale = 255.0
    acc_y_scale = 255.0
    acc_z_scale = 255.0

    g_scale = 9.81

    #<hyin/Jan-26-2015> known issue of the imu, yaw will drift when we rotate along roll
    #about 10-40 degree error, lets have a linear compensation here...
    #note there seems to be even more complicated coupling between pitch & roll, and pitch & yaw
    #but these seems some here cancel each other, need further investigation
    yawComp = 0.0
    bUseYawComp = False

    #publisher
    pub = rospy.Publisher(topic, Imu, queue_size=10)
    #service to receive command
    srv = rospy.Service(service, imu_cmd, imu_cmd_handler)
    #init node
    rospy.init_node('razor_imu_node', anonymous=True)
    rate = rospy.Rate(50)   #50Hz

    #a thread to detect key pressing
    #keythread = KeyEventThread()
    msg = Imu() 

    while not rospy.is_shutdown():
        if not bSync:
            #synchronize the reading...
            rospy.loginfo('RAZOR_IMU_NODE: Trying to setup and synch Razor...')
            #wait 3 seconds
            time.sleep(3)
            ser.write('#osab')  #binary mode for all calibrated sensor readings...
            ser.write('#o1')    #continous streaming
            ser.write('#oe0')   #disbale error message output

            #request a synch token
            ser.flushInput()
            ser.flushOutput()
            ser.write('#s00')

            #wait synchronization
            bMatch = False
            bMiss = False
            while not bMatch:
                #check the serial port
                if ser.inWaiting() < len(syn_token):
                    continue
                else:
                    #now check the reading against token
                    for i in range(len(syn_token)):
                        if ser.read(1) != syn_token[i]:
                            bMiss = True
                            break
                    if bMiss:
                        bMiss = False   #need a recheck...
                    else:
                        bMatch = True
            bSync = True
            #start the keythread
            #keythread.start()
            rospy.loginfo('RAZOR_IMU_NODE: Initializing...')

        #now we can read the information...
        #angles: 12 bytes accel: 12 bytes
        if ser.inWaiting() >= 24:
            yaw = np.radians(read_float(ser))
            pitch = np.radians(read_float(ser))
            roll = np.radians(read_float(ser))
            tmp_acc_x = read_float(ser)
            tmp_acc_y = read_float(ser)
            tmp_acc_z = read_float(ser)
            # print tmp_acc_x
            # print tmp_acc_y
            # print tmp_acc_z

            if bUseYawComp:
                #minus the compensation, yaw ~ (-pi, pi)
                yawComp = np.abs(roll) / (np.pi / 2) * np.radians(40.0) * (-1)
            else:
                yawComp = 0.0
            print 'ypr:', np.degrees([yaw-yawOffset, pitch, roll])
            #use local one
            # acc_x = tmp_acc_x
            # acc_y = tmp_acc_y
            # acc_z = tmp_acc_z

            #convert to global reference frame
            ori_mat = tftrans.euler_matrix(roll, pitch, yaw-yawOffset-yawComp, 'rxyz')
            accel_global = ori_mat.dot(np.array([tmp_acc_x, tmp_acc_y, tmp_acc_z, 1]))
            acc_x = accel_global[0]
            acc_y = accel_global[1]
            acc_z = accel_global[2]
        else:
            continue

        if not bZero:
            if zero_timer_cnt < num_zero_time:
                #accumulate accels
                acc_x_zero += acc_x
                acc_y_zero += acc_y
                acc_z_zero += acc_z
                zero_timer_cnt += 1
            else:
                #calculate zero level
                acc_x_zero = acc_x_zero / num_zero_time
                acc_y_zero = acc_y_zero / num_zero_time
                acc_z_zero = acc_z_zero / num_zero_time
                bZero = True
                acc_z_scale = acc_z_zero    #use this as the full amplitude gravity reading for z
                yawOffset = yaw
                rospy.loginfo('Streaming...')
            #do not send out message for this time period
            continue
        #test
        #print yaw, pitch, roll, acc_x, acc_y, acc_z
        #pack a message
        #from ypr to quaternion - downside + z, rotating frame - zyx
        #with lock:
        quat = tftrans.quaternion_from_euler(roll, pitch, yaw-yawOffset-yawComp, 'rzyx')

        #orientation, defined in external reference frame
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]

        #accel, defined in local sensor reference frame
        # msg.linear_acceleration.x = acc_x
        # msg.linear_acceleration.y = acc_y
        # msg.linear_acceleration.z = acc_z
        msg.linear_acceleration.x = -(acc_x - acc_x_zero) / acc_x_scale * g_scale
        msg.linear_acceleration.y = -(acc_y - acc_y_zero) / acc_y_scale * g_scale
        msg.linear_acceleration.z = -(acc_z - acc_z_zero) / acc_z_scale * g_scale

        msg.header.stamp = rospy.Time.now()

        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()