#!/usr/bin/env python
import rospy
import cv2
import time
from std_msgs.msg import String
import string
import numpy as np
#import Tkinter
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
allcoors=[[0.0,0.0]]
#ax.set_zlabel("z")

#fig=plt.figure()
#axes1=fig.add_subplot(111)
#line=axes1.plot(np.random.rand(10));
#def update(data):
#   line.set_ydata(data)
#   return line
def read_file(filename):
    content=""
    with open(filename,"a+") as fr:
         fr.seek(0)
         content=fr.read()
         print content
         print type(content)
    return content
def update(data):
    line.set_ydata(data)
    return line,
def data_gen():
    while True:
          plt.ion()
          line,=update(np.random.rand(10))
          plt.pause(0.5)
          plt.ioff()
def update_data(data):
    global allcoors
 #   data=read_file("./KeyFrameTrajectory.txt")
    strs=data.split("/")
    length=len(strs)
    label=0;
    for coor in strs:
        label=label+1;
        if label!=length:
           subtotal=[]
           substrs=coor.split(" ")
           for subcoor in substrs:
               subfloat=float(subcoor)
               subtotal.append(subfloat)
           if subtotal[1]>0.00001:
              temp=point(np.mat(subtotal[4:8]),subtotal[1],subtotal[2],subtotal[3])
              allcoors.append(temp)
    print allcoors
def point(pose,px,py,pz):
    """
    Project a 3D point into the camera.
    
    Input:
    pose -- camera pose
    px,py,pz -- point in global frame
    
    Output:
    u,v -- pixel coordinates
    
    """
    focalLength = 525.0
    centerX = 319.5
    centerY = 239.5

    q0=pose[0,0]
    q1=pose[0,1]
    q2=pose[0,2]
    q3=pose[0,3]
    R=[[1-2*q2*q2-2*q3*q3,2*q1*q2-2*q0*q3,2*q1*q3+2*q0*q2],
       [2*q1*q2+2*q0*q3,1-2*q1*q1-2*q3*q3,2*q2*q3-2*q0*q1],
       [2*q1*q3-2*q0*q2,2*q2*q3+2*q0*q1,1-2*q1*q1-2*q2*q2]]

    p = np.mat(R).T.dot(np.matrix([[px],[py],[pz]]))
    X = p[0,0]
    Y = p[1,0]
    Z = p[2,0]
    u = X/Z * 535.4 + 320.1
    v = Y/Z * 539.2 + 247.6
    return [u,v]

def draw_trajectory():
    global allcoors
    fig=plt.figure()
    ax=fig.add_subplot(111)
    while True:
          print "hsyhsy"
          plt.ion()	
          mats=np.mat(allcoors)
          ax.set_xlabel('X')
          ax.set_xlim(-2000, 2000)
          ax.set_ylabel('Y')
          ax.set_ylim(-2000, 2000)
  #  ax.set_zlabel('Z')
 #   ax.set_zlim(-1, 1)
#    ax.plot_wireframe(np.array(mats[:,1]),np.array(mats[:,2]),np.array(mats[:,3]))

          ax.plot(np.array(mats[:,0]),np.array(mats[:,1]),c="r")
          plt.pause(0.5)
          plt.ioff()
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    text=data.data
    update_data(text)

'''    #coors=text.isplit("/")
    print text
    plt.ion()
    X = np.linspace(-2 * np.pi, 2 * np.pi, 1000)
    y1 = np.sin(X)
    y2 = np.cos(X)
    plt.plot(X, y1, color='r', linestyle='--', linewidth=2, alpha=0.8)
    plt.plot(X, y2, color='b', linestyle='-', linewidth=2)
    plt.pause(0.5)
    plt.ioff()
'''

def listener():
    threading.Thread(target=draw_trajectory).start()
     # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("filePub", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    
