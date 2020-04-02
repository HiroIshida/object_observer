#!/usr/bin/env python

from object_observer.msg import *
import rospy 
import numpy as np
from jsk_recognition_msgs.msg import *

global box
global cloud
cloud = None
box = None


def callback_cloud(msg):
    global cloud
    X = np.array(msg.x_array.data)
    Y = np.array(msg.y_array.data)
    Z = np.array(msg.z_array.data)
    cloud = np.vstack((X, Y, Z)).T

def box_filter(box, cloud):
    b_min = box['pos'] - box['size'] * 0.5
    b_max = b_min + box['size']

    logical_x = (b_min[0] < cloud[:, 0]) * (cloud[:, 0] < b_max[0])
    logical_y = (b_min[1] < cloud[:, 1]) * (cloud[:, 1] < b_max[1])
    logical_z = (b_min[2] < cloud[:, 2]) * (cloud[:, 2] < b_max[2])
    logical = logical_x * logical_y * logical_z
    return logical


def callback_bba(msg):
    print("hoge")
    boxes_msg = msg.boxes

    def boxmsg2box(boxmsg):
        pose = boxmsg.pose
        pos_, ori = pose.position, pose.orientation
        dims = boxmsg.dimensions
        pos = np.array([pos_.x, pos_.y, pos_.z])
        size = np.array([dims.x, dims.y, dims.z])
        box = {'pos': pos, 'size': size}
        return box
    box_list = map(boxmsg2box, boxes_msg)

    def predicate(box):
        size2d = np.prod(box['size'][:2])
        logical_size = (size2d < 0.01)
        logical_height = (box['pos'][2] < 0.9)
        return logical_size * logical_height

    box_filtered = filter(predicate, box_list)
    print(len(box_filtered))



if __name__=='__main__':
    rospy.init_node("tmp", anonymous = True)
    sub = rospy.Subscriber('/vector_cloud', Cloud, callback_cloud)
    sub = rospy.Subscriber('/core/boxes', BoundingBoxArray, callback_bba)
    rospy.spin()

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    indices = box_filter(box, cloud)
    cloud = cloud[indices, :]

    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2])



