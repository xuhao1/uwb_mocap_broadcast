#include "uart_odom.h"
//#include "serial/serial.h"
#include "../../serial/include/serial/serial.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>

nav_msgs::Odometry m_init_odom;
nav_msgs::Odometry m_current_odom;

int main( int argc, char *argv[] )
{
    printf( "Hello, this is uart odom \n" );
    //    printf()
    cout << "Size of odom \r\n"
         << sizeof( nav_msgs::Odometry ) << endl;
    ros::init( argc, argv, "uart_odom" );
    ros::NodeHandle nh = ros::NodeHandle( "~" );
    //ros::NodeHandle nh( "~" );

    Uart_odom uart_odom( nh );

    uart_odom.test_uart();
    ros::MultiThreadedSpinner spinner( 6 ); // Use 4 threads
    spinner.spin();
    return 0;
}
