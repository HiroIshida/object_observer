#!/usr/bin/env python

from object_observer.msg import *
import rospy 
import numpy as np
from jsk_recognition_msgs.msg import *
from copy import deepcopy
import time
import scipy.spatial

def box_filter(box, cloud, margin=0.0):
    size = box['size'] * (1 + margin)
    b_min = box['pos'] - size * 0.5
    b_max = b_min + size

    logical_x = (b_min[0] < cloud[:, 0]) * (cloud[:, 0] < b_max[0])
    logical_y = (b_min[1] < cloud[:, 1]) * (cloud[:, 1] < b_max[1])
    logical_z = (b_min[2] < cloud[:, 2]) * (cloud[:, 2] < b_max[2])
    logical = logical_x * logical_y * logical_z
    return logical

def sequencial_box_filter(box_list, cloud, margin=0.0):
    cloud_decomposed_list = []
    cloud_remaining = deepcopy(cloud)
    for box in box_list:
        logical_indices = box_filter(box, cloud_remaining, margin)
        cloud_decomposed_list.append(cloud_remaining[logical_indices, :]) 
        cloud_remaining = cloud_remaining[~logical_indices, :]
    return cloud_decomposed_list

def publish_object_state(cvhull):
    print(cvhull.area)
    state = 'standing' if cvhull.area < 0.01 else 'falling'
    print(state)

class ObjectObserver:
    def __init__(self):
        self.cloud = None
        self.sub1 = rospy.Subscriber('/vector_cloud', Cloud, self.callback_cloud)
        self.sub2 = rospy.Subscriber('/core/boxes', BoundingBoxArray, self.callback_bba)

        # filed for debuggin
        self.cloud_list = None
        self.cvhull_list = None

    def callback_cloud(self, msg):
        X = np.array(msg.x_array.data)
        Y = np.array(msg.y_array.data)
        Z = np.array(msg.z_array.data)
        self.cloud = np.vstack((X, Y, Z)).T

    def callback_bba(self, msg):
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

        if self.cloud is not None:
            cloud_list = sequencial_box_filter(box_list, self.cloud, margin = 0.5)
            cvhull_list = [scipy.spatial.ConvexHull(X) for X in cloud_list]
            area_list = [cvhull.area for cvhull in cvhull_list]

            self.cloud_list = cloud_list
            self.cvhull_list = cvhull_list

            valid_idxes = []
            n_box = len(box_list)
            for i in range(n_box):
                if box_list[i]['pos'][2] < 0.95 and area_list[i] < 0.03:
                    valid_idxes.append(i)

            if len(valid_idxes)==1:
                cvhull = cvhull_list[valid_idxes[0]]
                publish_object_state(cvhull)








if __name__=='__main__':
    rospy.init_node("tmp", anonymous = True)
    oo = ObjectObserver()
    rospy.spin()

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    def show():
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        scat = lambda X, c: ax.scatter(X[:, 0], X[:, 1], X[:, 2], c=c)
        scat(oo.cloud_list[0], 'red')
        scat(oo.cloud_list[1], 'blue')

    show()



