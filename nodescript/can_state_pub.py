#!/usr/bin/env python

from object_observer.msg import *
import rospy 
import numpy as np
from jsk_recognition_msgs.msg import *
from jsk_rviz_plugins.msg import * 
from copy import deepcopy
import time
import scipy.spatial
import tf
from tf.transformations import quaternion_from_euler
import utils

class AverageQueue:
    def __init__(self, N):
        self.N = N
        self.data = []

    def push(self, elem):
        self.data.append(elem)
        if len(self.data) > self.N:
            self.data = self.data[1:]

    def isValid(self):
        return len(self.data) == self.N

    def mean(self):
        if self.isValid():
            return sum(self.data)/self.N
        else:
            return None

    def clear(self):
        self.data = []

def box_filter(box, cloud, margin=0.0):
    size = box['size'] * (1 + margin)
    b_min = box['pos'] - size * 0.5
    b_max = b_min + size

    logical_x = (b_min[0] < cloud[:, 0]) * (cloud[:, 0] < b_max[0])
    logical_y = (b_min[1] < cloud[:, 1]) * (cloud[:, 1] < b_max[1])
    logical = logical_x * logical_y 
    return logical

def sequencial_box_filter(box_list, cloud, margin=0.0):
    cloud_decomposed_list = []
    cloud_remaining = deepcopy(cloud)
    for box in box_list:
        logical_indices = box_filter(box, cloud_remaining, margin)
        cloud_decomposed_list.append(cloud_remaining[logical_indices, :]) 
        cloud_remaining = cloud_remaining[~logical_indices, :]
    return cloud_decomposed_list


class ObjectObserver:
    def __init__(self):
        self.cloud = None
        self.sub1 = rospy.Subscriber('/vector_cloud', Cloud, self.callback_cloud)
        self.sub2 = rospy.Subscriber('/core/boxes', BoundingBoxArray, self.callback_bba)

        self.pub_text = rospy.Publisher('object_status', OverlayText, queue_size=1)
        self.pub_statuspose = rospy.Publisher('object_statuspose', StatusPose2d, queue_size=1)

        self.br = tf.TransformBroadcaster()

        # filed for debuggin
        self.cloud_list = None
        self.cvhull2d_list = None
        self.aveque = AverageQueue(5) # only for filtering standing position

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
            cvhull2d_list = [scipy.spatial.ConvexHull(X[:, 0:2]) for X in cloud_list]
            area_list = [cvhull2d.area for cvhull2d in cvhull2d_list]
            self.cloud_list = cloud_list

            valid_idxes = []
            n_box = len(box_list)
            for i in range(n_box):
                # falling
                if box_list[i]['pos'][2] < 0.8 and area_list[i] < 0.6:
                    valid_idxes.append(i)
                # standing
                elif box_list[i]['pos'][2] < 0.9 and area_list[i] < 0.25:
                    valid_idxes.append(i)
            existObject = len(valid_idxes)==1

            if existObject:
                cvhull2d = cvhull2d_list[valid_idxes[0]]
                cloud = cloud_list[valid_idxes[0]]
                self.publish_object_state_ifexist(cvhull2d, cloud)
            else:
                self.publish_object_state_ifnotexist()

    def publish_object_state_ifexist(self, cvhull2d, cloud):
        center = list(np.mean(cloud, axis=0))

        state = 'standing' if cvhull2d.area < 0.25 else 'falling'
        text_status = OverlayText(text=state)
        self.pub_text.publish(text_status)

        if state is 'falling':
            self.aveque.clear()
            hull_points = cloud[:, 0:2][cvhull2d.vertices]
            rect = utils.minimum_bounding_rectangle(hull_points)
            theta = utils.get_rotation_angle(rect)
            q = quaternion_from_euler(0, 0, theta)
            self.br.sendTransform(center, list(q), rospy.Time.now(), "can", "base_footprint")

            sp = StatusPose2d(x=center[0], y=center[1], theta=theta, status=state)
            self.pub_statuspose.publish(sp)
        else: # standing
            self.aveque.push(np.array(center))
            center_average = self.aveque.mean()
            if center_average is None: 
                # if the queue is not filled, then nothing will be published
                return 
            center_average = center_average.tolist()
            sp = StatusPose2d(x=center_average[0], y=center_average[1], theta=0, status=state)
            center_average = self.aveque.mean().tolist()
            self.br.sendTransform(center_average, [0, 0, 0, 1], rospy.Time.now(), "can", "base_footprint")
            sp = StatusPose2d(x=center_average[0], y=center_average[1], theta=0, status=state)
            self.pub_statuspose.publish(sp)

    def publish_object_state_ifnotexist(self):
        self.aveque.clear()
        text_status = OverlayText(text="missing")
        self.pub_text.publish(text_status)


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




