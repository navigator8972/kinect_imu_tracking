#!/usr/bin/env python
"""
A module for fusing sensory input from IMU and Kinect IR camera
"""
import sys
import threading

import numpy as np

import rospy
import roslib; roslib.load_manifest('razor_imu_pkg')
from razor_imu_pkg.srv import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
import tf.transformations as tftrans

ir_topic = '/kinect_ir_node/ir_blob_pnt'
imu_topic = '/imu_node/data'
imu_srv = '/imu_node/cmd'

tip_pos_topic = '/kinect_ir_imu_fusion/tip_pos'
fusion_srv = '/kinect_ir_imu_fusion/cmd'

ir_data = None
imu_data = None

fusion_data_buffer = []

lock = threading.Lock()

def ir_callback(msg):
    global ir_data
    # rospy.loginfo('FUSION: Received reading from Kinect IR node')
    with lock:
        ir_data = msg
    return

def imu_callback(msg):
    global imu_data
    # rospy.loginfo('FUSION: Received reading from IMU node')
    with lock:
        imu_data = msg
    return

def fusion_cmd_handler(req):
    global imu_srv
    if req.cmd == 'align':
        #forward this to imu yaw alignment
        align_func = rospy.ServiceProxy(imu_srv, imu_cmd)
        align_func(req)
        rospy.loginfo('FUSION: Yaw aligned.')
    elif req.cmd == 'calibration':
        rospy.loginfo('TODO: A mode for calibration')
        pass
    return

def main(argv):
    #the first argument is the file path of script
    #check if necessary arguments are given
    if len(argv) <=4:
        print 'Not enough arguments. Run roslaunch?'
        return
    dist_cfg = float(argv[1])
    focal_cfg = float(argv[2])
    length_cfg = float(argv[3])

    global ir_topic, imu_topic, imu_srv, tip_pos_topic
    global ir_data, imu_data, fusion_data_buffer
    #initialize ros node
    rospy.init_node('kinect_ir_imu_fusion', anonymous=True)
    sub_ir = rospy.Subscriber(ir_topic, PointStamped, ir_callback)
    sub_imu = rospy.Subscriber(imu_topic, Imu, imu_callback)

    pub = rospy.Publisher(tip_pos_topic, PointStamped, queue_size=10)
    srv = rospy.Service(fusion_srv, imu_cmd, fusion_cmd_handler)
    tip_pos_msg = None

    rate = rospy.Rate(50)
    sync = False

    kinect_focal    = focal_cfg                 #unit: pixels - uncalibrated
    surf_dist       = dist_cfg                  #unit: mm
    barrel_len      = length_cfg                #unit: mm
    while not rospy.is_shutdown():
        if not sync:
            if ir_data is not None and imu_data is not None:
                sync = True
                rospy.loginfo('FUSION: Topics are ready...')
                #prepare message
                tip_pos_msg = PointStamped()
            rate.sleep()
            continue
        #get current data
        with lock:
            ir_x =  ir_data.point.x
            ir_y = ir_data.point.y
            imu_ori_x = imu_data.orientation.x
            imu_ori_y = imu_data.orientation.y
            imu_ori_z = imu_data.orientation.z
            imu_ori_w = imu_data.orientation.w

        #estimate tip position by fusing data
        #the infrared LED must be aligned to +x direction/FTDI connection of IMU
        #<hyin/Mar-30th-2015> the above comment is obsoleted. To avoid the alignment and drift
        #of yaw direction, make yaw direction align with the barrel axis
        #thus, now the infrared LED must be pointed in the -z direction
        quat = np.array([imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w])
        roll, pitch, yaw = tftrans.euler_from_quaternion(quat, 'rzyx')
        #print 'rpy:', np.degrees([roll, pitch, yaw])
        #ori_mat = tftrans.euler_matrix(roll, -pitch, -yaw, 'rzyx')
        ori_mat = tftrans.euler_matrix(-yaw, -pitch, roll, 'rzyx')
        #print ori_mat.dot(np.array([-barrel_len, 0, 0, 1]))
        #z = ori_mat.dot(np.array([-barrel_len, 0, 0, 1]))
        z = ori_mat.dot(np.array([0, 0, -barrel_len, 1]))
        #directions: +x: facing direction; +y: left hand direction +z: vertical direction pointing above
        pos = np.array([ir_x, ir_y]) * (surf_dist + z[2]) / kinect_focal
        print 'projection of z:', z
        print 'light_pos:', pos
        #<hyin/Mar-27th-2015> note here some efforts are required to deal with the orientation
        #of kinect with respect to the IMU coordinate
        #by placing the longtidual direction of kinect along x-axis and basement on the +y
        #the question is that the image is actually mirror of the motion direction along x-axis
        #thus revert the compensation along x...
        pos += np.array([-1, 1])*z[0:2]
        print 'tip_pos:', pos

        fusion_data_buffer.append(pos)
        if len(fusion_data_buffer) > 5:
            fusion_data_buffer = fusion_data_buffer[1:]

        tip_pos = np.average(np.array(fusion_data_buffer), axis=0)

        tip_pos_msg.header.stamp = rospy.Time.now()
        tip_pos_msg.point.x = tip_pos[0]
        tip_pos_msg.point.y = tip_pos[1]

        pub.publish(tip_pos_msg)
        #send this message
        rate.sleep()

    return


if __name__ == '__main__':
    # print 'Number of arguments:', len(sys.argv), 'arguments.'
    # print 'Argument List:', str(sys.argv)  
    main(sys.argv)