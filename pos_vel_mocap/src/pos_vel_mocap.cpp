#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <swarm_msgs/Pose.h>
#include <swarm_msgs/swarm_fused.h>
#include <swarm_msgs/swarm_fused_relative.h>
#include <swarm_msgs/swarm_drone_basecoor.h>

#define IF_SUBTRACT_INIT 1
using namespace std;
using namespace Eigen;

ros::Publisher pub_odom;
ros::Publisher pub_odom_slow;
ros::Publisher pub_path;
ros::Publisher pub_swarm_fused;
ros::Publisher pub_swarm_drone_basecoor;
bool enable_pub_path = false;

int pose_count = 0;
int self_id = 0;

bool init_ok = false;

Eigen::Vector3d    init_P, last_P;
Eigen::Quaterniond init_Q, last_Q, vins_Q;
ros::Time          now_t, last_odom_t, last_path_t;
Eigen::Vector3d    Vi0, Vi1, Vi2, Vi3, Vi4, Vo0;

nav_msgs::Path run_path;

std::map<int, Swarm::Pose> poses_vicon;
std::map<int, Swarm::Pose> poses_vicon_init;

void pose_callback( const geometry_msgs::PoseStamped & msg, int drone_id)
{
    ROS_INFO_THROTTLE(1.0, "[POS_VEL] Pose from %d received", drone_id);
    poses_vicon[drone_id] = Swarm::Pose(msg.pose);
    if (poses_vicon_init.find(drone_id)==poses_vicon_init.end()) {
        poses_vicon_init[drone_id] = Swarm::Pose(msg.pose);
    }
}

void pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg )
{
    ROS_INFO_THROTTLE(1.0, "[POS_VEL] Pose from %d received", self_id);
    poses_vicon[self_id] = Swarm::Pose(msg->pose);

    if (!init_ok) {
        poses_vicon_init[self_id] = Swarm::Pose(msg->pose);
        printf("Init self pose");
        poses_vicon_init[self_id].print();
    }

    if ( !init_ok )
    {
        init_ok = true;
        init_Q.w() = msg->pose.orientation.w;
        init_Q.x() = msg->pose.orientation.x;
        init_Q.y() = msg->pose.orientation.y;
        init_Q.z() = msg->pose.orientation.z;
        init_P.x() = msg->pose.position.x;
        init_P.y() = msg->pose.position.y;
        init_P.z() = msg->pose.position.z;
        last_P = init_P;
        last_Q = init_Q;
        last_odom_t = msg->header.stamp;
    }
    else
    {
        now_t = msg->header.stamp;

        Eigen::Vector3d    now_P, P_w;
        Eigen::Quaterniond now_Q, Q_w;
        now_P.x() = msg->pose.position.x;
        now_P.y() = msg->pose.position.y;
        now_P.z() = msg->pose.position.z;
        now_Q.w() = msg->pose.orientation.w;
        now_Q.x() = msg->pose.orientation.x;
        now_Q.y() = msg->pose.orientation.y;
        now_Q.z() = msg->pose.orientation.z;

        //std::cout << "x :" << msg->pose.position.x << "y:" << msg->pose.position.y
        //          << " z :" << msg->pose.position.z << std::endl;
        // Q_w = init_Q.normalized().toRotationMatrix().transpose() *
        // now_Q.normalized().toRotationMatrix();
        Q_w = now_Q.normalized().toRotationMatrix();
        if(IF_SUBTRACT_INIT)
        {
            P_w = now_P - init_P;
        }
        else
        {
            P_w = now_P;
        }

        Eigen::Vector3d now_vel;

        if((now_t-last_odom_t).toSec()>0.001)
        {
            now_vel = ( P_w - last_P ) / ( now_t - last_odom_t ).toSec();
            //        std::cout << " time " << ( now_t - last_t ).toSec( ) << std::endl;
            //        std::cout << " now_vel " << now_vel << std::endl;

            /** velocity filter **/
            if( (now_vel - Vi0).norm() / (now_t-last_odom_t).toSec() > 20.0 )
            {
                //printf("Vel error\n");
            }
            else
            {
                Vi0 = now_vel;
                Vo0 = ( Vi0 + Vi1 + Vi2 + Vi3 + Vi4 ) * 0.2;
                Vi4 = Vi3;
                Vi3 = Vi2;
                Vi2 = Vi1;
                Vi1 = Vi0;
                last_odom_t = now_t;
                last_P = P_w;
                last_Q = Q_w;
            }
        }


        /*********************/
        nav_msgs::Odometry odom;
        odom.header.stamp = now_t;
        odom.header.frame_id = "world";
        odom.pose.pose.position.x = P_w.x();
        odom.pose.pose.position.y = P_w.y();
        odom.pose.pose.position.z = P_w.z();
        odom.pose.pose.orientation.w = Q_w.w();
        odom.pose.pose.orientation.x = Q_w.x();
        odom.pose.pose.orientation.y = Q_w.y();
        odom.pose.pose.orientation.z = Q_w.z();
        odom.twist.twist.linear.x = Vo0.x(); // now_vel.x();
        odom.twist.twist.linear.y = Vo0.y(); // now_vel.y();
        odom.twist.twist.linear.z = Vo0.z(); // now_vel.z();
        pub_odom.publish( odom );

        if (pose_count ++ % 5 == 0) {
            pub_odom_slow.publish( odom );
        }

        if (enable_pub_path) {
            ros::Duration delta_t = now_t - last_path_t;
            if ( delta_t.toSec() > 0.1 )
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = now_t;
                pose.header.frame_id = "world";
                pose.pose.orientation.x = odom.pose.pose.orientation.x;
                pose.pose.orientation.y = odom.pose.pose.orientation.y;
                pose.pose.orientation.z = odom.pose.pose.orientation.z;
                pose.pose.orientation.w = odom.pose.pose.orientation.w;
                pose.pose.position.x = odom.pose.pose.position.x;
                pose.pose.position.y = odom.pose.pose.position.y;
                pose.pose.position.z = odom.pose.pose.position.z;

                run_path.header.stamp = now_t;
                run_path.header.frame_id = "world";
                run_path.poses.push_back( pose );
                pub_path.publish( run_path );

                last_path_t = now_t;
            }
        }
    }
}

void timer_callback(const ros::TimerEvent & e) {
    //Time to publish swarm stuffs..
    if (poses_vicon.find(self_id)==poses_vicon.end() || !init_ok) {
        ROS_INFO_THROTTLE(1.0, "[POS_VEL] timer_callback wait for %d", self_id);
        return;
    }


    swarm_msgs::swarm_drone_basecoor basecoor;
    auto self_init_pose = poses_vicon_init[self_id];
    basecoor.self_id = self_id;
    for (auto & it : poses_vicon_init) {
        int _drone_id = it.first;
        auto _pose = it.second;
        auto base_coor_pose = Swarm::Pose::DeltaPose(self_init_pose, _pose, true);

        geometry_msgs::Point p;
        p.x = base_coor_pose.pos().x();
        p.y = base_coor_pose.pos().y();
        p.z = base_coor_pose.pos().z();

        basecoor.ids.push_back(_drone_id);
        basecoor.drone_basecoor.push_back(p);
        basecoor.drone_baseyaw.push_back(base_coor_pose.yaw());

        geometry_msgs::Vector3 pos_cov;
        pos_cov.x = 0.01;
        pos_cov.y = 0.01;
        pos_cov.z = 0.01;
        basecoor.position_cov.push_back(pos_cov);
        basecoor.yaw_cov.push_back(0.01);
    }

    swarm_msgs::swarm_fused swarm_fused;

    for (auto & it : poses_vicon) {
        int _drone_id = it.first;
        auto _pose = it.second;
        _pose = Swarm::Pose::DeltaPose(self_init_pose, _pose, true);

        //Construct swarm_fused
        swarm_fused.ids.push_back(_drone_id);
        geometry_msgs::Point pt;
        pt.x = _pose.pos().x();
        pt.y = _pose.pos().y();
        pt.z = _pose.pos().z();
        swarm_fused.local_drone_position.push_back(pt);

        //TODO: TEMP set zero velocity
        geometry_msgs::Vector3 vel;
        vel.x = 0;
        vel.y = 0;
        vel.z = 0;
        swarm_fused.local_drone_velocity.push_back(vel);
        
        geometry_msgs::Vector3 pos_cov;
        pos_cov.x = 0.01;
        pos_cov.y = 0.01;
        pos_cov.z = 0.01;
        swarm_fused.position_cov.push_back(pos_cov);
        swarm_fused.yaw_cov.push_back(0.01);

        swarm_fused.local_drone_yaw.push_back(_pose.yaw());
    }

    auto _pose = poses_vicon[self_id];
    geometry_msgs::Point pt;
    pt.x = _pose.pos().x();
    pt.y = _pose.pos().y();
    pt.z = _pose.pos().z();
    swarm_fused.self_pos = pt;
    swarm_fused.self_yaw = poses_vicon[self_id].yaw();

    pub_swarm_fused.publish(swarm_fused);
    pub_swarm_drone_basecoor.publish(basecoor);
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "pos_vel_mocap" );
    ros::NodeHandle n( "~" );
    int drone_num = 0;
    n.param<int>("self_id", self_id, 1);
    n.param<int>("drone_num", drone_num, 10);
    
    char pose_topic[100] = {0};
    sprintf(pose_topic, "/SwarmNode%d/pose", self_id);
    ROS_INFO("Subscribe pose %s", pose_topic);
    ros::Subscriber s1 = n.subscribe(pose_topic , 100, pose_callback, ros::TransportHints().tcpNoDelay(true));

    std::vector<ros::Subscriber> subs;
    std::vector<boost::function<void (geometry_msgs::PoseStamped)>> cbs;
    for (unsigned int i = 0; i < drone_num; i ++) {
        if (i == self_id) {
            continue;
        }

        char pose_topic[100] = {0};
        sprintf(pose_topic, "/SwarmNode%d/pose", i);
        ROS_INFO("Subscribe pose %s", pose_topic);
        boost::function<void (geometry_msgs::PoseStamped)> cb = [=](geometry_msgs::PoseStamped pose) {
                pose_callback(pose, i); 
        };
        cbs.push_back(cb);
        auto hint = ros::TransportHints().tcpNoDelay(true);
        subs.push_back(n.subscribe<geometry_msgs::PoseStamped>(pose_topic, 100, cbs.back(), ros::VoidConstPtr(), hint));
    }

    pub_odom_slow = n.advertise< nav_msgs::Odometry >( "/vins_estimator/odometry", 100 );
    pub_odom = n.advertise< nav_msgs::Odometry >( "/vins_estimator/imu_propagate", 100 );
    pub_swarm_fused = n.advertise< swarm_msgs::swarm_fused >( "/swarm_drones/swarm_drone_fused", 100 );
    pub_swarm_drone_basecoor = n.advertise< swarm_msgs::swarm_drone_basecoor >( "/swarm_drones/swarm_drone_basecoor", 100 );

    ros::Timer timer = n.createTimer(ros::Duration(0.01), timer_callback);
    ros::spin();
    return 0;
}
