#!/usr/bin/env python

from object_observer.msg import *
import rospy 
import numpy as np

global cloud
cloud = None


def callback_cloud(msg):
    global cloud
    X = np.array(msg.x_array.data)
    Y = np.array(msg.y_array.data)
    Z = np.array(msg.z_array.data)
    cloud = np.vstack((X, Y, Z)).T

def callback_bba(msg):
    boxes = msg.boxes
    if len(boxes) == 1:
        pose = boxes[0].pose
        pos, ori = pose.position, pose.orientation
        pos2d = np.array([pos.x, pos.y])
        queue.push(pos2d)

if __name__=='__main__':
    rospy.init_node("tmp", anonymous = True)
    sub = rospy.Subscriber('/vector_cloud', Cloud, callback_cloud)
    #sub = rospy.Subscriber('/core/boxes', BoundingBoxArray, callback_bba)
    rospy.spin()

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2])



