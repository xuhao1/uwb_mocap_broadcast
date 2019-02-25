// The mini odom is aim at to minimize the size of ros::odom, from 760 to 74 (using float)
// If using eigen half struct, the size can be reduce to 34
#ifndef MINI_ODOM_H
#define MINI_ODOM_H

#define IF_HALF_MINI_ODOM 0
// if using half odom, the packet size can be reduce to 34 .


#include <nav_msgs/Odometry.h>
#include <stddef.h>

#if IF_HALF_MINI_ODOM
#include <Eigen/Eigen>
//#include "Half.h"
#include <Eigen/src/Core/arch/CUDA/Half.h>
#endif

//--- Odom example
//header:
//  seq: 13351
//  stamp:
//    secs: 1544989199
//    nsecs: 231717066
//  frame_id: "world"
//child_frame_id: ''
//pose:
//  pose:
//    position:
//      x: -0.000209818910491
//      y: -0.0299552707378
//      z: 0.000343322753906
//    orientation:
//      x: -0.0914082815531
//      y: 6.89346711602e-07
//      z: 2.10519029725e-06
//      w: 0.995813499637
//  covariance: [0.4444444444444444, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4444444444444444, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4444444444444444, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
//twist:
//  twist:
//    linear:
//      x: -1.16860894997e-06
//      y: -0.12243012952
//      z: -0.000534057617188
//    angular:
//      x: 0.0
//      y: 0.0
//      z: 0.0
//  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
//---

template <typename T, typename T2>
struct Mini_odom
{
    T2 seq;
    char frame_id[20];
    T pos_x;
    T pos_y;
    T pos_z;
    T linear_spd_x;
    T linear_spd_y;
    T linear_spd_z;

    T q_x;
    T q_y;
    T q_z;
    T q_w;

    T angle_x;
    T angle_y;
    T angle_z;
    T angle_spd_x;
    T angle_spd_y;
    T angle_spd_z;
};

template <typename T, typename T2>
void odom_to_miniodom(nav_msgs::Odometry & odom ,Mini_odom<T, T2> & mini_odom )
{
    mini_odom.seq = odom.header.seq;
    //mini_odom.frame_id = odom.header.frame_id;
    memcpy(&mini_odom.frame_id, &odom.header.frame_id, std::string(odom.header.frame_id).length() );
    mini_odom.pos_x = odom.pose.pose.position.x;
    mini_odom.pos_y = odom.pose.pose.position.y;
    mini_odom.pos_z = odom.pose.pose.position.z;

    mini_odom.q_x = odom.pose.pose.orientation.x;
    mini_odom.q_y = odom.pose.pose.orientation.y;
    mini_odom.q_z = odom.pose.pose.orientation.z;
    mini_odom.q_w = odom.pose.pose.orientation.w;

    mini_odom.linear_spd_x = odom.twist.twist.linear.x;
    mini_odom.linear_spd_y = odom.twist.twist.linear.y;
    mini_odom.linear_spd_z = odom.twist.twist.linear.z;

    mini_odom.angle_spd_x = odom.twist.twist.angular.x;
    mini_odom.angle_spd_y = odom.twist.twist.angular.y;
    mini_odom.angle_spd_z = odom.twist.twist.angular.z;

}

template <typename T, typename T2>
void miniodom_to_odom(Mini_odom<T, T2> & mini_odom,  nav_msgs::Odometry & odom )
{
    odom.header.seq = mini_odom.seq;
    odom.header.stamp = ros::Time::now();

    odom.pose.pose.position.x = mini_odom.pos_x;
    odom.pose.pose.position.y = mini_odom.pos_y;
    odom.pose.pose.position.z = mini_odom.pos_z;

    odom.pose.pose.orientation.x = mini_odom.q_x ;
    odom.pose.pose.orientation.y = mini_odom.q_y ;
    odom.pose.pose.orientation.z = mini_odom.q_z ;
    odom.pose.pose.orientation.w = mini_odom.q_w ;

    odom.twist.twist.linear.x = mini_odom.linear_spd_x;
    odom.twist.twist.linear.y = mini_odom.linear_spd_y;
    odom.twist.twist.linear.z = mini_odom.linear_spd_z;

    odom.twist.twist.angular.x = mini_odom.angle_spd_x ;
    odom.twist.twist.angular.y = mini_odom.angle_spd_y ;
    odom.twist.twist.angular.z = mini_odom.angle_spd_z ;

}

#endif // MINI_ODOM_H
