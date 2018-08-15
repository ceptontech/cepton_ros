#!/usr/bin/env python

import json

import numpy

import rospy
import tf


class TransformsNode(object):
    def __init__(self):
        rospy.init_node("cepton_transforms")
        self.rate = rospy.Rate(rospy.get_param("~rate", 10))
        self.transform_broadcaster = tf.TransformBroadcaster()

        self.ros_namespace = rospy.get_param("~namespace", "cepton")
        self.parent_frame_id = rospy.get_param("~parent_frame_id", "world")

        self.transforms_dict = {}
        file_path = rospy.get_param("~file_path", "")
        if file_path:
            with open(file_path, "r") as transforms_file:
                self.transforms_dict = json.load(transforms_file)

    def publish_transforms(self):
        for serial_number, transform_dict in self.transforms_dict.items():
            translation = transform_dict.get("translation", numpy.zeros(3))
            rotation = transform_dict.get(
                "rotation", numpy.array([0.0, 0.0, 0.0, 1.0]))
            frame_id = "{}_{}".format(self.ros_namespace, serial_number)
            self.transform_broadcaster.sendTransform(
                translation, rotation, rospy.Time.now(), frame_id, self.parent_frame_id)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_transforms()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = TransformsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
