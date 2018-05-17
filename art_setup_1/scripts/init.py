#!/usr/bin/env python

import sys
import rospy
from art_utils import ArtApiHelper
from art_msgs.msg import CollisionPrimitive
from shape_msgs.msg import SolidPrimitive
import os

setup = None


def makePrimitive(name, dim, x=None, y=None, z=None, ori=(0, 0, 0, 1), frame_id="marker"):

    global setup

    pr = CollisionPrimitive()
    pr.name = name
    pr.setup = setup
    pr.bbox.type = SolidPrimitive.BOX
    pr.bbox.dimensions = dim
    pr.pose.header.frame_id = frame_id

    pr.pose.pose.orientation.x = ori[0]
    pr.pose.pose.orientation.y = ori[1]
    pr.pose.pose.orientation.z = ori[2]
    pr.pose.pose.orientation.w = ori[3]

    if x is None:
        pr.pose.pose.position.x = pr.bbox.dimensions[0] / 2
    else:
        pr.pose.pose.position.x = x

    if y is None:
        pr.pose.pose.position.y = pr.bbox.dimensions[1] / 2
    else:
        pr.pose.pose.position.y = y

    if z is None:
        pr.pose.pose.position.z = pr.bbox.dimensions[2] / 2
    else:
        pr.pose.pose.position.z = z

    return pr


def main(args):

    global setup

    rospy.init_node('setup_init_script', anonymous=True)

    try:
        setup = os.environ["ARTABLE_SETUP"]
    except KeyError:
        rospy.logerr("ARTABLE_SETUP has to be set!")
        return

    api = ArtApiHelper()
    api.wait_for_db_api()

    rospy.loginfo("Refreshing collision environment...")
    api.clear_collision_primitives(setup)

    table_width = 1.5
    table_depth = 0.7
    api.add_collision_primitive(makePrimitive("table", [table_width, table_depth, 0.78], z=-0.78 / 2))

    feeder_depth = 0.35
    feeder_thickness = 0.001
    feeder_front_to_table = 0.15

    # left feeder (1)
    api.add_collision_primitive(
        makePrimitive("lf-front", [feeder_depth, feeder_thickness, 0.2], x=-feeder_depth / 2 - feeder_front_to_table, y=table_depth-0.495))
    api.add_collision_primitive(
        makePrimitive("lf-middle", [feeder_depth, feeder_thickness, 0.39], x=-feeder_depth / 2 - feeder_front_to_table,
                      y=table_depth-0.18))
    api.add_collision_primitive(
        makePrimitive("lf-rear", [feeder_depth, feeder_thickness, 0.39], x=-feeder_depth / 2 - feeder_front_to_table,
                      y=table_depth))

    # right feeder (2)
    api.add_collision_primitive(makePrimitive(
        "rf-front", [feeder_depth, feeder_thickness, 0.175], x=table_width + feeder_depth / 2 + feeder_front_to_table, y=table_depth-0.495))
    api.add_collision_primitive(
        makePrimitive("rf-middle", [feeder_depth, feeder_thickness, 0.35], x=table_width + feeder_depth / 2 + feeder_front_to_table,
                      y=table_depth-0.18))
    api.add_collision_primitive(
        makePrimitive("rf-rear", [feeder_depth, feeder_thickness, 0.35], x=table_width + feeder_depth / 2 + feeder_front_to_table,
                      y=table_depth))

    rospy.loginfo("Done!")


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
