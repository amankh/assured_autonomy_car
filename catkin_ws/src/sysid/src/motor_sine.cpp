// Motor sine to determine the delay time in motor
// command inputs a sinusoidal wave 
// system logs both the motor input and the wheel velocity

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/transform_datatypes.h>

#include <sstream>
#include <fstream>
#include <cmath>

#define PI 3.1415926

//using namespace std;

// global variables
// parameters
double steering_gain, steering_offset;
// values to be logged, updated in cb funcs
double steering_ang;
double long_vel, lat_vel, yaw_rate;
double yaw_rate_imu, yaw_rate_dot, lat_acc;
double wheel_vel;
ackermann_msgs::AckermannDriveStamped cmd;

// void servoCb(const std_msgs::Float64& msg)
// {
//     steering_ang = ( msg.data - steering_offset ) / steering_gain;
// }

// void poseCb(const nav_msgs::Odometry& msg)
// {
//     long_vel = msg.twist.twist.linear.x;
//     lat_vel = msg.twist.twist.linear.y;
//     yaw_rate = msg.twist.twist.angular.z;
// }

void odomCb(const nav_msgs::Odometry& msg)
{
    wheel_vel = msg.twist.twist.linear.x/0.03235;
}

// void imuCb(const sensor_msgs::Imu& msg)
// {
//     ros::Time curr_time = ros::Time::now();
//     static ros::Time last_time = curr_time;
    
//     yaw_rate_imu = msg.angular_velocity.z;
//     static double last_yaw_rate = yaw_rate_imu;
    
//     yaw_rate_dot = (yaw_rate - last_yaw_rate) / (curr_time - last_time).toSec();
//     lat_acc = msg.linear_acceleration.y;

//     last_yaw_rate = yaw_rate;
//     last_time = curr_time;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_sine");
    ros::NodeHandle nh;

//    ros::Subscriber steer_sub = nh.subscribe("/sensors/servo_position_command",1,servoCb);
//    ros::Subscriber pose_sub = nh.subscribe("/ekf_localization/odom",1,poseCb);
//    ros::Subscriber imu_sub = nh.subscribe("/imu",1,imuCb);
    ros::Subscriber odom_sub = nh.subscribe("/odom_fused",1,odomCb);
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/keyboard",1);

    std::ofstream outstream;
    outstream.open("/home/nvidia/Desktop/motor_sine.csv");


    ros::Duration(5).sleep();          // wait for vesc driver to initialize
    ros::Rate log_rate(20);
    // cmd.drive.speed = 3;
    
    ros::Time start_time = ros::Time::now();
    ros::Duration diff = ros::Time::now() - start_time;

    double frequency = 1.0;     // Hz, sinusoidal test period
    double max_speed = 2.0;     // sinusoidal amplitude

    while(diff.toSec() <= 15.0 && ros::ok()) {
//        ros::Time log_start_time = ros::Time::now();
        
        double vel = max_speed * sin(2*PI*frequency*(diff.toSec()));
        cmd.drive.speed = vel;
        cmd.drive.steering_angle = 0.0;
        cmd_pub.publish(cmd);
//        outstream << vel << "," << wheel_vel << std::endl ;

        ROS_INFO("Running at wheel velocity of %lf", wheel_vel);
//        ros::Time log_start_time = ros::Time::now();
//        while ( (ros::Time::now() - log_start_time).toSec() < 1.0 && ros::ok() ) {
//            ros::spinOnce();
//            outstream << ros::Time::now().toSec() << "," << cmd.drive.speed << "," << wheel_vel << std::endl ;
//            cmd_pub.publish(cmd);
//            log_rate.sleep();
//        }
        ros::spinOnce();
        outstream << diff.toSec() << "," << cmd.drive.speed << "," << wheel_vel << std::endl ;
        cmd_pub.publish(cmd);
        log_rate.sleep();

        diff = ros::Time::now() - start_time;
    }



    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    cmd_pub.publish(cmd);
    outstream.close();
    return 0;
}
