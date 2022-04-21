#!/usr/bin/env python
import rospy
from slam_msgs.msg import StateWithCovariance

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse
import threading
from Error_Function import ErrorFunction

class PltPoint():
    def __init__(self,x,y,covx,covy,color,label,ax):
        self.label = label
        self.x = x
        self.y = y
        self.covx = covx
        self.covy = covy
        self.color = color+'o'
        self.point, = plt.plot(self.x,self.y,self.color,ms=10)
        self.ellipse = Ellipse(xy=(x,y),width=2*covx,height=2*covy,angle=0)
        self.ellipse.set_alpha(0.5)
        self.ellipse.set_facecolor(color)
        ax.add_patch(self.ellipse)
    
    def setPoint(self,x,y):
        self.x = x
        self.y = y
        self.ellipse.center = (x,y)

    def setCovariance(self,covx,covy):
        self.covx = covx
        self.covy = covy
        self.ellipse.width = 2*covx
        self.ellipse.height = 2*covy

    def redraw(self):
        self.remove()
        self.point, = plt.plot(self.x,self.y,self.color)

    def remove_all(self):
        self.ellipse.width = 0
        self.ellipse.height = 0
        self.point.remove()

    def remove(self):
        self.point.remove()
    
    def __str__(self):
        s = str(self.label)+' POINT2D '+str(self.label)+' '
        s += str(self.x) + ' ' + str(self.y) +'\n \n'
        return s
        

class RobotLine(PltPoint):
    def __init__(self, ax):
        self.robot = PltPoint(0,0,1,1,'b',0,ax)
        self.line_data = [[0.],[0.]]
        self.arrow_data = 0.
        self.line, = plt.plot(self.line_data[0],self.line_data[1],'b-')
        self.arrow = ax.arrow(0, 0, 0.3, 0., head_width=0.05, head_length=0.05, fc='k', ec='k')
        self.ax = ax
    
    def setPoint(self,x,y,theta):
        self.robot.setPoint(x,y)
        self.arrow_data = theta
        self.line_data[0].append(x)
        self.line_data[1].append(y)

    def setCovariance(self,covx,covy):
        self.robot.setCovariance(covx,covy)

    def remove(self):
        self.robot.remove_all()
        self.line.remove()
        self.arrow.remove()

    def redraw(self):
        self.robot.redraw()
        self.arrow.remove()
        self.line.remove()
        self.line, = plt.plot(self.line_data[0],self.line_data[1],'b-')
        self.arrow = self.ax.arrow(self.robot.x, self.robot.y, 0.3*np.cos(self.arrow_data), 0.3*np.sin(self.arrow_data), head_width=0.05, head_length=0.05, fc='k', ec='k')


class MapPainter:

    def callbackState(self,msg):
        n = int(len(msg.covariance)**(0.5))
        self.lock.acquire()
        robotxy = msg.state[0:3]

        cov = np.reshape(np.array(msg.covariance),(n,n))

        self.robot.setPoint(msg.state[0],msg.state[1], msg.state[2])
        self.robot.setCovariance(cov[0][0],cov[1][1])
        self.robot.redraw()

        

        if (len(msg.state)-3)/2 < len(self.lms):
            for lm in self.lms:
                lm.remove()
            self.lms = []

        if msg.header.seq < self.input_seq:
            for i in range(len(self.lms)):
                self.lms[i].remove_all()
            self.robot.remove()
            self.lms = []
            self.robot = RobotLine(self.ax)

        self.input_seq = msg.header.seq

        i = 3
        j = 0
        while i+1 < len(msg.state):
            if j < len(self.lms):
                self.lms[j].setPoint(msg.state[i],msg.state[i+1])
                self.lms[j].setCovariance(cov[i][i],cov[i+1][i+1])
                self.lms[j].redraw()
            else:
                lm = PltPoint(msg.state[i],msg.state[i+1],cov[i][i],cov[i+1][i+1],'r',msg.seenLandmarks[j],self.ax)
                print("I have seen landmark ",msg.seenLandmarks[j])
                self.lms.append(lm)
            i += 2
            j += 1

        self.lock.release()

        # Write landmarks to file
        f = open(self.fname,'w')
        lms_ordered = self.lms
        lms_ordered = sorted(lms_ordered, key=lambda lm : lm.label)
        for lm in lms_ordered:
            f.write(str(lm))
        f.close()
        new_err = ErrorFunction(self.fname, 'gt.txt')
        if abs(self.err - new_err) > 0.01:
            self.err = new_err
            #print "Solution error:", self.err
        

    def __init__(self):
        self.fname = 'SLAM_Output.txt'
        self.input_seq = 0
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-1, 4), ylim=(-1, 4))
        self.lms = []
        self.robot = RobotLine(self.ax)
        self.err = 0
        self.lock = threading.Lock()

        rospy.Subscriber('/SLAM_state_X/', StateWithCovariance, self.callbackState)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.lock.acquire()
            plt.pause(0.01)
            self.lock.release()
            r.sleep()

if __name__ == '__main__':
    print('SLAM Plotting Started...')
    # Initialize the node and name it.
    rospy.init_node('plotter')
    try:
        map_painter = MapPainter()
    except rospy.ROSInterruptException: pass
