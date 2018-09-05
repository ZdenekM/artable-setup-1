#!/usr/bin/env python

import sys
import rospy
from art_utils import ArtApiHelper
from art_msgs.msg import CollisionPrimitive
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
import os
from art_msgs.msg import Program, ProgramBlock, ProgramItem
from art_utils.art_msgs_functions import obj_type

setup = None

# TODO n1_kinect2_link, n2_kinect2_link (0.3)

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
    # api.add_collision_primitive(
    #     makePrimitive("lf-front", [feeder_depth, feeder_thickness, 0.2], x=-feeder_depth / 2 - feeder_front_to_table,
    # y=table_depth-0.495))
    # api.add_collision_primitive(
    #     makePrimitive("lf-middle", [feeder_depth, feeder_thickness, 0.39],
    # x=-feeder_depth / 2 - feeder_front_to_table,
    #                  y=table_depth-0.18))
    # api.add_collision_primitive(
    #     makePrimitive("lf-rear", [feeder_depth, feeder_thickness, 0.39], x=-feeder_depth / 2 - feeder_front_to_table,
    #                  y=table_depth))

    # right feeder (2)
    api.add_collision_primitive(makePrimitive(
        "rf-front", [feeder_depth, feeder_thickness, 0.175], x=table_width + feeder_depth / 2 + feeder_front_to_table, y=table_depth-0.495))
    api.add_collision_primitive(
        makePrimitive("rf-middle", [feeder_depth, feeder_thickness, 0.35], x=table_width + feeder_depth / 2 + feeder_front_to_table,
                      y=table_depth-0.18))
    api.add_collision_primitive(
        makePrimitive("rf-rear", [feeder_depth, feeder_thickness, 0.35], x=table_width + feeder_depth / 2 + feeder_front_to_table,
                      y=table_depth))

    api.add_collision_primitive(makePrimitive("kinect-n1", [0.3, 0.3, 0.3], x=0, y=0, z=0, frame_id="n1_kinect2_link"))
    api.add_collision_primitive(makePrimitive("kinect-n2", [0.3, 0.3, 0.3], x=0, y=0, z=0, frame_id="n2_kinect2_link"))

    api.store_object_type(obj_type("Stretcher", 0.046, 0.046, 0.154))
    api.store_object_type(obj_type("ShortLeg", 0.046, 0.046, 0.298))
    api.store_object_type(obj_type("LongLeg", 0.046, 0.046, 0.398))

    # delete all created programs
    ph = api.get_program_headers()
    if ph:
        for h in ph:
            api.program_clear_ro(h.id)
            api.delete_program(h.id)

    # TODO add parameters
    prog = Program()
    prog.header.id = 1
    prog.header.name = "MSV DEMO"

    pb = ProgramBlock()
    pb.id = 1
    pb.name = "Pick and place"
    pb.on_success = 2
    pb.on_failure = 0
    prog.blocks.append(pb)

    pi = ProgramItem()
    pi.id = 1
    pi.on_success = 2
    pi.on_failure = 0
    pi.type = "PickFromFeeder"
    pi.object.append("")
    pi.pose.append(PoseStamped())
    pb.items.append(pi)

    pi = ProgramItem()
    pi.id = 2
    pi.on_success = 3
    pi.on_failure = 0
    pi.type = "PlaceToPose"
    pi.pose.append(PoseStamped())
    pi.ref_id.append(1)
    pb.items.append(pi)

    pi = ProgramItem()
    pi.id = 3
    pi.on_success = 0
    pi.on_failure = 0
    pi.type = "GetReady"
    pb.items.append(pi)

    pb = ProgramBlock()
    pb.id = 2
    pb.name = "Inspect and sort"
    pb.on_success = 1
    pb.on_failure = 0
    prog.blocks.append(pb)

    pi = ProgramItem()
    pi.id = 1
    pi.on_success = 2
    pi.on_failure = 0
    pi.type = "PickFromPolygon"
    pi.object.append("")
    pi.polygon.append(PolygonStamped())
    pb.items.append(pi)

    pi = ProgramItem()
    pi.id = 2
    pi.on_success = 3
    pi.on_failure = 4
    pi.type = "VisualInspection"
    pi.pose.append(PoseStamped())
    pi.ref_id.append(1)
    pb.items.append(pi)

    pi = ProgramItem()
    pi.id = 3
    pi.on_success = 5
    pi.on_failure = 0
    pi.type = "PlaceToContainer"
    pi.name = "OK"
    pi.object.append("")
    pi.polygon.append(PolygonStamped())
    pi.ref_id.append(1)
    pb.items.append(pi)

    pi = ProgramItem()
    pi.id = 4
    pi.on_success = 5
    pi.on_failure = 0
    pi.type = "PlaceToContainer"
    pi.name = "NOK"
    pi.object.append("")
    pi.polygon.append(PolygonStamped())
    pi.ref_id.append(1)
    pb.items.append(pi)

    pi = ProgramItem()
    pi.id = 5
    pi.on_success = 0
    pi.on_failure = 0
    pi.type = "GetReady"
    pb.items.append(pi)

    api.store_program(prog)

    rospy.loginfo("Done!")


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
