#!/usr/bin/env python2
from __future__ import print_function

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from pymavlink4swarm import MAVLink
import pymavlink4swarm as pymavlink
from inf_uwb_ros.msg import data_buffer, incoming_broadcast_data

class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)


class UWB2Vicon:
    def __init__(self, vicon_node_id, nodes):

        f = fifo()
        self.mav = MAVLink(f)
        self.pubs = {}
        self.sub = rospy.Subscriber('/uwb_node/incoming_broadcast_data', incoming_broadcast_data, self.on_uwb_data,
                                    queue_size=10, tcp_nodelay=True)
        self.nodes = nodes
        self.vicon_node_id = vicon_node_id

        for k in nodes:
            pub = rospy.Publisher("/swarm_mocap/SwarmNodeOdom{}".format(k), Odometry, queue_size=1)
            self.pubs[k] = pub


    def parse_drone_odom_data(self, msg):
        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "world"

        node_id = msg.source_id
        x = msg.x / 1000.0
        y = msg.y / 1000.0
        z = msg.z / 1000.0

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z

        vx = msg.vx / 1000.0
        vy = msg.vy / 1000.0
        vz = msg.vz / 1000.0

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz

        qw = msg.q0 / 10000.0
        qx = msg.q1 / 10000.0
        qy = msg.q2 / 10000.0
        qz = msg.q3 / 10000.0

        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz


        rospy.loginfo_throttle(1.0, "node {} x {:4.3f} y {:4.3f} z{:4.3f} vx {:4.3f} {:4.3f} {:4.3f} q {:3.2f} {:3.2f} {:3.2f} {:3.2f}".format(
            node_id,
            x, y, z,
            vx, vy, vz,
            qw, qx, qy, qz
        ))

        if node_id in self.pubs:
            # print("Pub", node_id, self.pubs[node_id])
            self.pubs[node_id].publish(odom)

    def on_uwb_data(self, income_data):
        if income_data.remote_id == self.vicon_node_id:
            try:
                msgs = self.mav.parse_buffer(income_data.data)
                # print(msgs)
                for msg in msgs:
                    if msg.get_msgId() == pymavlink.MAVLINK_MSG_ID_DRONE_ODOM_GT:
                        self.parse_drone_odom_data(msg)

            except Exception as inst:
                rospy.logwarn_throttle(1.0, "Mavlink parse Error Warn {}".format(inst))

if __name__ == "__main__":
    print("Starting UART MULTI UWB")
    rospy.init_node("UWB2Vicon")
    uwb2vicon = UWB2Vicon(0, [1, 2, 3, 4, 5, 6, 7, 8, 9])
    rospy.spin()

