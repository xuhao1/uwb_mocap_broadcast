#!/usr/bin/env python2
from __future__ import print_function

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from pymavlink4swarm import MAVLink
from inf_uwb_ros.msg import data_buffer

class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)


class Vicon2UWB:
    def __init__(self, rate):

        f = fifo()
        self.mav = MAVLink(f)
        self.cbs = []

        self.pose_last = {}
        self.pub2uwb = rospy.Publisher('/uwb_node/send_broadcast_data', data_buffer, queue_size=1)
        self.last_send = {}
        self.dt = 1.0 / rate
        self.sub0 = rospy.Subscriber("/swarm_mocap/SwarmNodePose0", PoseStamped, self.pose_cb0, queue_size=1, tcp_nodelay=True)
        self.sub1 = rospy.Subscriber("/swarm_mocap/SwarmNodePose1", PoseStamped, self.pose_cb1, queue_size=1, tcp_nodelay=True)
        self.sub2 = rospy.Subscriber("/swarm_mocap/SwarmNodePose2", PoseStamped, self.pose_cb2, queue_size=1, tcp_nodelay=True)
        self.sub3 = rospy.Subscriber("/swarm_mocap/SwarmNodePose3", PoseStamped, self.pose_cb3, queue_size=1, tcp_nodelay=True)
        self.sub4 = rospy.Subscriber("/swarm_mocap/SwarmNodePose4", PoseStamped, self.pose_cb4, queue_size=1, tcp_nodelay=True)
        self.sub5 = rospy.Subscriber("/swarm_mocap/SwarmNodePose5", PoseStamped, self.pose_cb5, queue_size=1, tcp_nodelay=True)
        self.sub6 = rospy.Subscriber("/swarm_mocap/SwarmNodePose6", PoseStamped, self.pose_cb6, queue_size=1, tcp_nodelay=True)
        self.sub7 = rospy.Subscriber("/swarm_mocap/SwarmNodePose7", PoseStamped, self.pose_cb7, queue_size=1, tcp_nodelay=True)
        self.sub8 = rospy.Subscriber("/swarm_mocap/SwarmNodePose8", PoseStamped, self.pose_cb8, queue_size=1, tcp_nodelay=True)
        self.sub9 = rospy.Subscriber("/swarm_mocap/SwarmNodePose9", PoseStamped, self.pose_cb9, queue_size=1, tcp_nodelay=True)

    def pose_cb0(self, pose):
        self.pose_cb(0, pose)

    def pose_cb1(self, pose):
        self.pose_cb(1, pose)

    def pose_cb2(self, pose):
        self.pose_cb(2, pose)

    def pose_cb3(self, pose):
        self.pose_cb(3, pose)

    def pose_cb4(self, pose):
        self.pose_cb(4, pose)

    def pose_cb5(self, pose):
        self.pose_cb(5, pose)

    def pose_cb6(self, pose):
        self.pose_cb(6, pose)

    def pose_cb7(self, pose):
        self.pose_cb(7, pose)

    def pose_cb8(self, pose):
        self.pose_cb(8, pose)

    def pose_cb9(self, pose):
        self.pose_cb(9, pose)


    def pose_cb(self, id, pose):
        # print("On pose id {}".format(id))
        # odom = Odometry()
        # odom.pose.pose = pose.pose
        # odom.header.stamp = pose.stamp
        pos = pose.pose.position
        att = pose.pose.orientation
        # print(pose)

        if id not in self.pose_last:
            vel = Point(0, 0, 0)
            self.pose_last[id] = pose
        else:
            dt = (pose.header.stamp - self.pose_last[id].header.stamp).to_sec()
            pos_last = self.pose_last[id].pose.position
            vel = Point(
                (pos.x - pos_last.x) / dt,
                (pos.y - pos_last.y) / dt,
                (pos.z - pos_last.z) / dt
            )
            # print(vel, dt)
            self.pose_last[id] = pose

        msg = self.mav.drone_odom_gt_encode(id,
                int(pos.x*1000), int(pos.y*1000), int(pos.z*1000),
                int(att.w*10000), int(att.x*10000), int(att.y*10000), int(att.z*10000),
                int(vel.x*1000), int(vel.y*1000), int(vel.z*1000))


        if id not in self.last_send:
            self.last_send[id] = rospy.get_time()
            self.send_mavlink_msg(msg)
            return

        # print(rospy.get_time() - self.last_send[id])
        if rospy.get_time() - self.last_send[id] > self.dt:
            self.last_send[id] = rospy.get_time()
            # print("Send ", rospy.get_time() - self.last_send[id], self.dt)
            self.send_mavlink_msg(msg)

        rospy.loginfo_throttle(1.0, "ID {} pos {:4.3f} {:4.3f} {:4.3f} VEL {:4.3f} {:4.3f} {:4.3f}".format(
            id, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z))

        # print(msg)

    def send_mavlink_msg(self, msg):
        buf = msg.pack(self.mav, force_mavlink1=False)
        #Size 33
        _buf = data_buffer()
        _buf.data = buf

        # print(len(buf))
        self.pub2uwb.publish(_buf)



if __name__ == "__main__":
    print("Starting UART MULTI UWB")
    rospy.init_node("Vicon2UWB")
    vi2uwb = Vicon2UWB(50.0)

    rospy.spin()

