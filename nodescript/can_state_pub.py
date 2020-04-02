#!/usr/bin/env python

from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Point
import rospy 
from tf2_msgs.msg import TFMessage
import tf
import numpy as np

class MyQueue:
    def __init__(self, N):
        self.N = N
        self.data = [np.zeros(2) for n in range(N)]

    def push(self, elem):
        tmp = self.data[1:self.N]
        tmp.append(elem)
        self.data = tmp

    def mean(self):
        s_est_lst = [np.mean(np.array([s[i] for s in self.data])) for i in range(2)]
        return np.array(s_est_lst)

br = tf.TransformBroadcaster()
queue = MyQueue(5)
pub = rospy.Publisher("/object_position", Point, queue_size = 1)

def callback_tf(msg):
    x, y = queue.mean()
    z = 0.74
    trans = [x, y, z]
    rot = [0, 0, 0, 1]
    br.sendTransform(tuple(trans), tuple(rot), rospy.Time.now(), "object", "base_footprint")
    point_msg = Point(x = x, y = y, z = z)
    pub.publish(point_msg)

def callback_bba(msg):
    boxes = msg.boxes
    if len(boxes) == 1:
        pose = boxes[0].pose
        pos, ori = pose.position, pose.orientation
        pos2d = np.array([pos.x, pos.y])
        queue.push(pos2d)

if __name__=='__main__':
    rospy.init_node("coords_pub", anonymous = True)
    sub = rospy.Subscriber('/tf', TFMessage, callback_tf)
    sub = rospy.Subscriber('/core/boxes', BoundingBoxArray, callback_bba)
    rospy.spin()

