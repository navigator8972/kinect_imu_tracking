#!/usr/bin/env python
"""
An example to show how to receive estimated tip-position and visualize
trajectory on a white board...
"""
import sys
import threading

import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure
matplotlib.rc('xtick', labelsize=14) 
matplotlib.rc('ytick', labelsize=14) 

import rospy
from geometry_msgs.msg import PointStamped

class PyKinectIMUBoard(QMainWindow):
    def __init__(self, parent=None, inc_mode=False):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('KinectIMUBoard - PyQt4')
        self.resize(1024, 768)
        self.move(400, 200)

        self.max_data_lst_len_  = 50
        self.data_lst_          = []
        self.curr_tip_pos_      = None
        self.data_refreshed_    = False
        self.mode_incremental_  = inc_mode
        self.tip_pos_topic_     = '/kinect_ir_imu_fusion/tip_pos'

        self.lock_              = threading.Lock()

        self.create_main_frame()
        self.init_ros()
        self.main_proc()

    def tip_pos_callback(self, msg):
        with self.lock_:
            self.curr_tip_pos_ = np.array([msg.point.x, msg.point.y])
            self.data_refreshed_ = True
        return

    def create_main_frame(self):
        self.main_frame_ = QWidget()
        fig_vbox = QVBoxLayout()
        self.dpi_ = 100
        self.fig_ = plt.figure()
        self.canvas_ = FigureCanvas(self.fig_)
        self.ax_ = self.fig_.add_subplot(111)
        self.ax_pnts_ = None
        self.ax_.set_aspect('equal')
        self.ax_.autoscale(False, False, False)
        self.ax_.set_xlim([-100, 100])
        self.ax_.set_ylim([-100, 100])

        self.clear_btn_ = QPushButton('Clear')
        self.clear_btn_.clicked.connect(self.clear_plot)

        fig_vbox.addWidget(self.canvas_, 5)
        fig_vbox.addWidget(self.clear_btn_, 1)
        self.main_frame_.setLayout(fig_vbox)
        self.setCentralWidget(self.main_frame_)
        return

    def init_ros(self):
        #initialize ros node
        rospy.init_node('kinect_imu_board', anonymous=True)
        self.sub_ = rospy.Subscriber(self.tip_pos_topic_ , PointStamped, self.tip_pos_callback)

        # self.rate_=rospy.Rate(10)

    def main_proc(self):
        #prepare function to draw
        self.timer_ = QTimer()

        self.timer_.timeout.connect(self.plot_data)

        self.timer_.start(10)

    def clear_plot(self):
        self.ax_.clear()
        self.ax_.set_aspect('equal')
        self.ax_.autoscale(False, False, False)
        self.ax_.set_xlim([-100, 100])
        self.ax_.set_ylim([-100, 100])
        return

    def plot_data(self):
        # print 'Calling plot_data'
        #prepare data
        if self.mode_incremental_:
            if self.curr_tip_pos_ is not None:
                self.ax_pnts_, = self.ax_.plot(-self.curr_tip_pos_[1], -self.curr_tip_pos_[0], 'bo')
                self.ax_.hold(True)
                self.canvas_.draw()
        else:
            with self.lock_:
                if self.curr_tip_pos_ is not None and self.data_refreshed_:
                    self.data_lst_.append(self.curr_tip_pos_)
                    if len(self.data_lst_) > self.max_data_lst_len_:
                        self.data_lst_ = self.data_lst_[(len(self.data_lst_)-self.max_data_lst_len_):]
                    self.data_refreshed_ = False
                else:
                    return
        
            if len(self.data_lst_) >= 1:
                data_to_plot = -np.array(self.data_lst_)
                if self.ax_pnts_ is None:
                    self.ax_pnts_, = self.ax_.plot(data_to_plot[:, 1], data_to_plot[:, 0], 'bo')
                    self.ax_.hold(True)
                else:
                    #update
                    self.ax_pnts_.set_xdata(data_to_plot[:, 1])
                    self.ax_pnts_.set_ydata(data_to_plot[:, 0])
                self.canvas_.draw()
        return

def main():
    app = QApplication(sys.argv)
    board = PyKinectIMUBoard(inc_mode=True)
    board.show()
    app.exec_()
    return

if __name__ == '__main__':
    main()
