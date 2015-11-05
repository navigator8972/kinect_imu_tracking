"""
GUI to display imu readings
"""

import numpy as np 
import threading

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy
from sensor_msgs.msg import Imu 
import tf.transformations as tftrans

topic = '/imu_node/data'

#some globale variables to update and share with GUI
yaw = 0.0
pitch = 0.0
roll = 0.0
yawOffset = 0.0

acc_x = 0.0
acc_y = 0.0
acc_z = 0.0

#lock
lock = threading.Lock()

class razor_imu_gui(QMainWindow):

    def __init__(self):
        return

def spin_proc():
    rospy.spin()
    return

def imu_data_callback(imu_data):
    global yaw, pitch, roll, acc_x, acc_y, acc_z
    euler = tftrans.euler_from_quaternion(( imu_data.orientation.x, 
                                            imu_data.orientation.y,
                                            imu_data.orientation.z,
                                            imu_data.orientation.w), 'rzyx')
    with lock:
        yaw = euler[0]
        pitch = euler[1]
        roll = euler[2]
        acc_x = imu_data.linear_acceleration.x
        acc_y = imu_data.linear_acceleration.y
        acc_z = imu_data.linear_acceleration.z
    return

def main():
    #init node
    rospy.init_node('razor_imu_gui', anonymous=True)
    #subscriber
    sub = rospy.Subscriber(topic, Imu, imu_data_callback)
    #thread for spinning
    #<hyin/Jan-23-2015> Rate.sleep() will do the thing that spinonce can do, no more a thread for spin()
    #note this is different from roscpp
    #spin_thread = threading.Thread(target=spin_proc)
    #spin_thread_started = False

    #for the first two seconds, average the data for accel

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # if not spin_thread_started:
        #   spin_thread.start()
        #   spin_thread_started = True

        with lock:
            print 'Orientation:', yaw, pitch, roll
            print 'Acceleration:', acc_x, acc_y, acc_z

        rate.sleep()
    return

if __name__ == '__main__':
    main()