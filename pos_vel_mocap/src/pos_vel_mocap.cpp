#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#define IF_SUBTRACT_INIT 0
using namespace std;
using namespace Eigen;

ros::Publisher pub_odom;
ros::Publisher pub_path;

bool init_ok = false;

Eigen::Vector3d    init_P, last_P;
Eigen::Quaterniond init_Q, last_Q, vins_Q;
ros::Time          now_t, last_odom_t, last_path_t;
Eigen::Vector3d    Vi0, Vi1, Vi2, Vi3, Vi4, Vo0;

nav_msgs::Path run_path;

void pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg )
{
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


//        ros::Duration delta_t = now_t - last_path_t;
//        if ( delta_t.toSec() > 0.1 )
//        {
//            geometry_msgs::PoseStamped pose;
//            pose.header.stamp = now_t;
//            pose.header.frame_id = "world";
//            pose.pose.orientation.x = odom.pose.pose.orientation.x;
//            pose.pose.orientation.y = odom.pose.pose.orientation.y;
//            pose.pose.orientation.z = odom.pose.pose.orientation.z;
//            pose.pose.orientation.w = odom.pose.pose.orientation.w;
//            pose.pose.position.x = odom.pose.pose.position.x;
//            pose.pose.position.y = odom.pose.pose.position.y;
//            pose.pose.position.z = odom.pose.pose.position.z;

//            run_path.header.stamp = now_t;
//            run_path.header.frame_id = "world";
//            run_path.poses.push_back( pose );
//            pub_path.publish( run_path );

//            last_path_t = now_t;
//        }
    }
}

void vins_callback( const nav_msgs::Odometry::ConstPtr msg )
{
    vins_Q.w() = msg->pose.pose.orientation.w;
    vins_Q.x() = msg->pose.pose.orientation.x;
    vins_Q.y() = msg->pose.pose.orientation.y;
    vins_Q.z() = msg->pose.pose.orientation.z;

    Matrix3d vins_R = vins_Q.toRotationMatrix();
    Matrix3d now_R = last_Q.toRotationMatrix();

    Vector3d n = vins_R.col( 0 );
    Vector3d o = vins_R.col( 1 );
    Vector3d a = vins_R.col( 2 );

    double vin_y = atan2( n( 1 ), n( 0 ) );
    double vin_p = atan2( -n( 2 ), n( 0 ) * cos( vin_y ) + n( 1 ) * sin( vin_y ) );
    double vin_r = atan2( a( 0 ) * sin( vin_y ) - a( 1 ) * cos( vin_y ),
                          -o( 0 ) * sin( vin_y ) + o( 1 ) * cos( vin_y ) );

    n = vins_R.col( 0 );
    o = vins_R.col( 1 );
    a = vins_R.col( 2 );

    double now_y = atan2( n( 1 ), n( 0 ) );
    double now_p = atan2( -n( 2 ), n( 0 ) * cos( now_y ) + n( 1 ) * sin( now_y ) );
    double now_r = atan2( a( 0 ) * sin( now_y ) - a( 1 ) * cos( now_y ),
                          -o( 0 ) * sin( now_y ) + o( 1 ) * cos( now_y ) );

    std::cout << "compare\n"
              << "now_r :" << now_r << "\nvins_r:" << vin_r
              << "\n\n now_p :" << now_p << "\nvins_p:" << vin_p
              << "\n\nnow_y :" << now_y << "\nvins_y:" << vin_y << "\n\n\n\n";
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "pos_vel_mocap" );
    ros::NodeHandle n( "~" );

    ros::Subscriber s1 = n.subscribe( "/Robot_1/pose", 100, pose_callback );
    ros::Subscriber s2 =
        n.subscribe( "/vins_estimator/odometry", 10, vins_callback );

    pub_odom = n.advertise< nav_msgs::Odometry >( "odom_TA", 100 );
    //pub_path = n.advertise< nav_msgs::Path >( "/mocap_path", 10 );

    ros::spin();
    return 0;
}
