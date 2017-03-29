#!/usr/bin/env python

import json

import rospy
import tf

class TransformsNode(object):
    ROTATION_CONVERSION = {
        "euler": lambda x: tf.transformations.quaternion_from_euler(x[0], x[1], x[2]),
        "quaternion": lambda x: x
    }

    def __init__(self):
        rospy.init_node("cepton_transforms")
        self.rate = rospy.Rate(rospy.get_param("~rate", 10))
        self.transform_broadcaster = tf.TransformBroadcaster()

        self.parent_frame_id = rospy.get_param("~parent_frame_id", "world")
        self.name_prefix = rospy.get_param("~name_prefix", "cepton")

        self.transforms_dict = {}
        file_path = rospy.get_param("~file_path", "")
        if file_path:
            with open(file_path, "r") as transforms_file:
                self.transforms_dict = json.load(transforms_file)

    def publish_transforms(self):
        for sensor_name, transform in self.transforms_dict.items():
            translation = transform["translation"]

            if transform["rotation_type"] not in TransformsNode.ROTATION_CONVERSION:
                raise ValueError("invalid rotation_type: {}".format(
                    transform["rotation_type"]))
            rotation = \
                TransformsNode.ROTATION_CONVERSION[transform["rotation_type"]](
                    transform["rotation"])

            time = rospy.Time.now()
            frame_id = "cepton_{}".format(sensor_name)
            parent_frame = "world"

            self.transform_broadcaster.sendTransform(
                translation, rotation, time, frame_id, self.parent_frame_id)

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
