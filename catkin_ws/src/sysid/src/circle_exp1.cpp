// Steady state cornering test to determine cornering stiffness
// Based on Sierra, Tseng, Jain & Peng (2006)
// Plot I_zz*r_dot+m*L_r*a_y against m*L*a_y
// find least squares error line
// gradient gives X1, y-intercept gives L(delta-L*r/u)*X2
// C_alpha_r = X2/X1 ; C_alpha_f = X1/(1-X1)*C_alpha_r

// exp1 : steering angle 5 deg 
// exp1 : speed 1 m/s


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <sstream>
#include <fstream>

using namespace std;

// global variables
// parameters
double steering_gain, steering_offset;

// values to be logged, updated in callback funcs
double steering_ang;
double long_vel, lat_vel, yaw_rate;
double yaw_rate_imu, yaw_rate_dot, lat_acc;
double wheel_vel;
ackermann_msgs::AckermannDriveStamped cmd;

void servoCb(const std_msgs::Float64& msg)
{
    steering_ang = ( msg.data - steering_offset ) / steering_gain;
}

void poseCb(const nav_msgs::Odometry& msg)
{
    long_vel = msg.twist.twist.linear.x;
    lat_vel = msg.twist.twist.linear.y;
    yaw_rate = msg.twist.twist.angular.z;
}

void odomCb(const nav_msgs::Odometry& msg)
{
    wheel_vel = msg.twist.twist.linear.x/0.03235;
}

void imuCb(const sensor_msgs::Imu& msg)
{
    ros::Time curr_time = ros::Time::now();
    ros::Time last_time = curr_time;
    
    yaw_rate_imu = msg.angular_velocity.z;
    double last_yaw_rate = yaw_rate_imu;
    
    yaw_rate_dot = (yaw_rate - last_yaw_rate) / (curr_time - last_time).toSec(); 

    lat_acc = msg.linear_acceleration.y;

    last_yaw_rate = yaw_rate;
    last_time = curr_time;
}

int main(int argc, char **argv)
{
    double forward_vel = 2;
    int steering_ang_deg = -5;

    ros::init(argc, argv, "ramp_steer");
    ros::NodeHandle nh;

    if (!nh.getParam("vesc_interface/steering_gain",steering_gain)) {
        ROS_FATAL("Parameter steering_gain is required.");
        return 1;
    }
    if (!nh.getParam("vesc_interface/steering_offset", steering_offset)) {
        ROS_FATAL("Parameter steering_offset is required.");
        return 1;
    }

    ros::Subscriber steer_sub = nh.subscribe("/sensors/servo_position_command",1,servoCb);
    ros::Subscriber pose_sub = nh.subscribe("/ekf_localization/odom",1,poseCb);
    ros::Subscriber imu_sub = nh.subscribe("/imu",1,imuCb);
    ros::Subscriber odom_sub = nh.subscribe("/odom_fused",1,odomCb);
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/keyboard",1);

    // wait for vesc driver to initialize
    ros::Duration(5).sleep();

    // log rate at 20 Hz      
    ros::Rate log_rate(20);

    // longitudinal velocity
    cmd.drive.speed = forward_vel;
    
    // specify output file loction
    std::ofstream outfile;
    outfile.open("/home/nvidia/Desktop/Sysid/circle_exp1_trial1.csv");
    // left turn ramp steering test
    
    cmd.drive.speed = forward_vel;
    cmd.drive.steering_angle = steering_ang_deg*M_PI/180.0;

    cmd_pub.publish(cmd);
    ROS_INFO("circle_exp1: Running at steering angle %d degrees.\n", steering_ang_deg);
    ROS_INFO("circle_exp1: Running at forward velocity %lf .\n", forward_vel);

    ros::Time start_time = ros::Time::now();
    while ( (ros::Time::now() - start_time).toSec() < 30.0 && ros::ok() ) {
        //    
        ros::spinOnce();

        outfile << wheel_vel << "," << steering_ang << "," << long_vel << "," << lat_vel
        << "," << yaw_rate << "," << yaw_rate_imu << "," << yaw_rate_dot << "," << lat_acc << endl;

        cmd_pub.publish(cmd);
        log_rate.sleep();
    }
    

    outfile.close();
    ROS_INFO("circle_exp1: ending at forward velocity %lf .\n", forward_vel);

    // zero all parameters
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    cmd_pub.publish(cmd);
    return 0;
}
