// @brief: TO correct
// Sine steer test to determine cornering stiffness
// Based on Sierra, Tseng, Jain & Peng (2006)
// Plot I_zz*r_dot+m*L_r*a_y against m*L*a_y
// find least squares error line
// gradient gives X1, y-intercept gives L(delta-L*r/u)*X2
// C_alpha_r = X2/X1 ; C_alpha_f = X1/(1-X1)*C_alpha_r

// exp3 : steering angle from -30 to 0 
// exp3 : speed 2


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <sstream>
#include <fstream>


#include <math.h>
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
    double forward_vel = 0.0;

    ros::init(argc, argv, "sine_steer");
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
    double speed_amplitude = 0.5;
    double speed_offset = 1.0;
    double speed_frequency = 0.0;
    double steering_amplitude = 20;
    double steering_frequency = 0.1;
    double steering_ang_deg = 0;

    // input steering
    char cmd_c = 's';

    // time
    ros::Time start_time =ros::Time::now();
    ros::Duration diff =ros::Time::now() - start_time;
    
    // specify output file loction
    std::ofstream outfile;
    outfile.open("/home/nvidia/Desktop/Sysid/sine_steer_exp1_trial0.csv");
    ROS_INFO("sine_steer_exp1: Running at steering angle degrees.\n");

        
    //  ramp steering test
    while(diff.toSec()<100 && ros::ok())
    {

	forward_vel = speed_amplitude*sin(2*M_PI*speed_frequency*(diff.toSec())) + speed_offset;
	steering_ang_deg = steering_amplitude*sin(2*M_PI*steering_frequency*(diff.toSec()));

	steering_ang_deg =10.0;
//	if (steering_ang_deg < 0.0){
//	steering_ang_deg -= 5;
//	}
//	else {
//	steering_ang_deg +=5;
//	}
	scanf("%c", cmd_c);
	if(cmd_c == 'a'){
	steering_ang_deg +=5;
	ROS_INFO("sine_steer_exp1: Running at steering angle degrees: %f \n", steering_ang_deg);	
	cmd_c = 's';
	}
	else if (cmd_c == 'd'){
	steering_ang_deg -= 5;
	ROS_INFO("sine_steer_exp1: Running at steering angle degrees: %f \n", steering_ang_deg);
	cmd_c = 's';
	}
	

        cmd.drive.speed = forward_vel;
        cmd.drive.steering_angle = steering_ang_deg*M_PI/180.0;

        cmd_pub.publish(cmd);
        
        
            
        ros::spinOnce();
        outfile << wheel_vel << "," << steering_ang << "," << long_vel << "," << lat_vel
        << "," << yaw_rate << "," << yaw_rate_imu << "," << yaw_rate_dot << "," << lat_acc << endl;
        cmd_pub.publish(cmd);
        log_rate.sleep();
	
	diff = ros::Time::now() - start_time;
        
    }

    outfile.close();

    // zero all parameters
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    cmd_pub.publish(cmd);
    return 0;
}
