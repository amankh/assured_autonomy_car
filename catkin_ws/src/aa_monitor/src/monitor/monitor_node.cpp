/**
 * @author: edwardahn,smitsch
 */

#include <csetjmp>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_datatypes.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "std_msgs/builtin_double.h"

#include "aa_monitor/utils/utils.h"
#include "aa_monitor/utils/consts.h"

// For calling to/from CakeML
extern "C" int cml_main(int argc, char **argv);
extern "C" void cml_exit(void);
std::jmp_buf env;

// Expose FFIs to CakeML
extern "C" void fficonst(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffisense(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffictrl(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffiextCtrl(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffiactuate(char *c, long clen, int32_t *a, long alen);
extern "C" void ffistop(int32_t *c, long clen, int8_t *a, long alen);
extern "C" void ffiviolation(char *c, long clen, int32_t *a, long alen);

// ROS message types
typedef ackermann_msgs::AckermannDriveStamped action_t;
typedef nav_msgs::Odometry state_t;
typedef geometry_msgs::Point point_t;
typedef diagnostic_msgs::DiagnosticStatus diag_status_t;
typedef diagnostic_msgs::DiagnosticArray diag_array_t;
typedef diagnostic_msgs::KeyValue kv_t;
typedef geometry_msgs::PointStamped monitor_t;

// Queue size for ROS subscribers
const size_t QUEUE_SIZE = 1;

// Publishers with safe actions and monitor verdicts
static ros::Publisher safe_action_pub;
static ros::Publisher monitor_diagnostic_pub;
static ros::Publisher monitor_verdict_pub;

// Save subscribed messages for processing later
static double current_state_timestamp = -1;
static double last_state_timestamp = -1;
static std::vector<double> current_state;
static std::vector<double> current_converted_state;
static std::vector<double> last_converted_state;
static std::vector<double> current_waypoint;
static std::vector<double> proposed_action;

void printConsts(int A, int B, int T, int eps) {
    printf("Consts  A=%d,B=%d,T=%d,eps=%d\n", A, B, T, eps);
}

void printSensors(int t, int v, int xg, int yg) {
    printf("Sensors t=%d,v=%d,xg=%d,yg=%d\n",t,v,xg,yg);
}

void printCtrl(int a, int k, int t, int vh, int vl, int xg, int yg) {
    printf("Ctrl    a=%d,k=%d,t=%d,vh=%d,vl=%d,xg=%d,yg=%d\n",a,k,t,vh,vl,xg,yg);
}

void printActuate(int v, int s) {
    printf("Act     v=%d,s=%d\n",v,s);
}

void fillKV(const char* key, double val, diag_status_t &status) {
    char buf[128];
    sprintf(buf, "%f", val);
    kv_t kv;
    kv.key = key;
    kv.value = buf;
    status.values.push_back(kv);
}

void fficonst(int32_t *c, long clen, int32_t *a, long alen) {
    assert(clen == 0);
    assert(alen == 4 * 4);

    a[0] = MAX_MOTOR_ACCEL; // maximum acceleration A [dm/s^2]
    a[1] = MAX_MOTOR_ACCEL; // maximum acceleration B [dm/s^2]
    a[2] = T_CYCLE_TIME;    // control cycle duration T [ds]
    a[3] = 1.0;             // trajectory tolerance eps [m]
    printConsts(a[0],a[1],a[2],a[3]);
}

void ffisense(int32_t *c, long clen, int32_t *a, long alen) {
    assert(clen == 0);
    assert(alen == 4 * 4);

    printf("Sensing...\n");

    ros::spinOnce();

    while (current_state_timestamp == last_state_timestamp) {
      // convert [ds]->[s] and wait 1/10th of T_CYCLE_TIME
      ros::Duration(T_CYCLE_TIME/100.0).sleep();
      ros::spinOnce();
    }

    last_converted_state = current_converted_state;
    std::vector<double> sense_state = current_state;
    sense_state.insert(sense_state.end(), current_waypoint.begin(), current_waypoint.end());
    current_converted_state.clear();
    convertState(sense_state, current_converted_state);
    
    // monitor expects (t,v,xg,yg)
    a[0] = (current_state_timestamp-last_state_timestamp)*10.0;   // t [ds]
    a[1] = current_converted_state[4];             // v [dm/s]
    a[2] = current_converted_state[7];             // xg [dm]
    a[3] = current_converted_state[8];             // yg [dm]

    printSensors(a[0],a[1],a[2],a[3]);
    
    diag_status_t new_status;
    new_status.level = diag_status_t::OK;
    new_status.message = "Converted sensors";
    new_status.hardware_id = "ModelPlex";
    new_status.name = "Veriphy monitor";
    fillKV("t [ds]",   a[0], new_status);
    fillKV("v [dm/s]", a[1], new_status);
    fillKV("xg [dm]",  a[2], new_status);
    fillKV("yg [dm]",  a[3], new_status);
    diag_array_t new_diag;
    new_diag.status.push_back(new_status);
    monitor_diagnostic_pub.publish(new_diag);
}

void ffiextCtrl(int32_t *c, long clen, int32_t *a, long alen) {
    //  assert(clen == 11 * 4);
    assert(alen == 7 * 4);

    printf("Ext Ctrl...\n");

    // reading from aa_monitor subscriptions here
    // 1) read from aa_monitor/action here
    // 2) convert actions to Brandon's form
    // 3) Set control values as below
    long double time = current_state_timestamp - last_state_timestamp;
    long double acc = time > 0.0L && last_converted_state.size() > 0 ? (current_converted_state[4] - last_converted_state[4])/time : 0.0L;

    std::vector<double> converted_action;
    std::vector<double> ctrl_action = proposed_action;
    ctrl_action.push_back(acc);
    ctrl_action.push_back(10.0);  // vh
    ctrl_action.push_back(0.0);   // vl
    ctrl_action.push_back(current_converted_state[7]);
    ctrl_action.push_back(current_converted_state[8]);
    ctrl_action.push_back(current_converted_state[9]);
    convertAction(ctrl_action, converted_action);

    // monitor expects (a,k,t,vh,vl,xg,yg)
    a[0] = converted_action[0]; // a [dm/s^2]
    a[1] = converted_action[1]; // k [centi-(meters^-1)]
    a[2] = converted_action[2]; // t [ds]
    a[3] = converted_action[3]; // vh [dm/s]
    a[4] = converted_action[4]; // vl [dm/s]
    a[5] = converted_action[5]; // xg [dm]
    a[6] = converted_action[6]; // yg [dm]
    
    printCtrl(a[0],a[1],a[2],a[3],a[4],a[5],a[6]);

    diag_status_t new_status;
    new_status.level = diag_status_t::OK;
    new_status.message = "Converted external control";
    new_status.hardware_id = "ModelPlex";
    new_status.name = "VeriPhy monitor";
    fillKV("a [dm/s^2]", a[0], new_status);
    fillKV("k [centi-(meters^-1)]", a[1], new_status);
    fillKV("t [ds]", a[2], new_status);
    fillKV("vh [dm/s]", a[3], new_status);
    fillKV("vl [dm/s]", a[4], new_status);
    fillKV("xg [dm]", a[5], new_status);
    fillKV("yg [dm]", a[6], new_status);
    diag_array_t new_diag;
    new_diag.status.push_back(new_status);
    monitor_diagnostic_pub.publish(new_diag);
}

void ffictrl(int32_t *c, long clen, int32_t *a, long alen) {
    ffiextCtrl(c,clen,a,alen);
}

void ffiactuate(char *c, long clen, int32_t *a, long alen) {
    assert(alen == 7 * 4);

    printf("Actuate...\n");

    const char* how = (const char *)c; // distinguish between normal OK and fallback
    
    diag_status_t new_status;    
    if (strncmp(how,"OK",clen) == 0) {
        new_status.level = diag_status_t::OK;
        new_status.message = how;
        new_status.hardware_id = "ModelPlex";
        new_status.name = "VeriPhy monitor";
        fillKV("a [dm/s^2]", a[0], new_status);
        fillKV("k [centi-(meters^-1)]", a[1], new_status);
        fillKV("t [ds]", a[2], new_status);
        fillKV("vh [dm/s]", a[3], new_status);
        fillKV("vl [dm/s]", a[4], new_status);
        fillKV("xg [dm]", a[5], new_status);
        fillKV("yg [dm]", a[6], new_status);

        monitor_t verdict_msg;
        verdict_msg.header.stamp = current_state_timestamp > 0 ? ros::Time(current_state_timestamp) : ros::Time();
        verdict_msg.point.x = 1;
        verdict_msg.point.y = 1;
        monitor_verdict_pub.publish(verdict_msg);
    } else if (strncmp(how,"Control Violation",clen) == 0) {
        new_status.level = diag_status_t::WARN;
        new_status.message = how;
        new_status.hardware_id = "ModelPlex";
        new_status.name = "VeriPhy monitor";
        fillKV("a [dm/s^2]", a[0], new_status);
        fillKV("k [centi-(meters^-1)]", a[1], new_status);
        fillKV("t [ds]", a[2], new_status);
        fillKV("vh [dm/s]", a[3], new_status);
        fillKV("vl [dm/s]", a[4], new_status);
        fillKV("xg [dm]", a[5], new_status);
        fillKV("yg [dm]", a[6], new_status);
        
        printf("Control violation a=%d, k=%d, t=%d, vh=%d, vl=%d, xg=%d, yg=%d\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6]);

        monitor_t verdict_msg;
        verdict_msg.header.stamp = current_state_timestamp > 0 ? ros::Time(current_state_timestamp) : ros::Time();
        verdict_msg.point.x = -1;
        verdict_msg.point.y = -1;
        monitor_verdict_pub.publish(verdict_msg);
    } else {
        printf("HUH\n");
        // Unknown string -- should never occur
        assert(false);
    }
    diag_array_t new_diag;
    new_diag.status.push_back(new_status);
    monitor_diagnostic_pub.publish(new_diag);
    
    double vset = proposed_action[0]; // fmax(current_state[5] + a[0]*T_CYCLE_TIME/100.0, 0.0); // converts [dm/s^2]*[ds] to [m/s]
    double steer = proposed_action[1];     // fallback does not change steering

    printActuate(vset, steer);

    // publish to commands/keyboard
    action_t new_action;
    new_action.drive.speed = vset;
    new_action.drive.steering_angle = steer;
    safe_action_pub.publish(new_action);
    
    // make ffisense wait for next measurement
    last_state_timestamp = current_state_timestamp;
}

void ffistop(int32_t *c, long clen, int8_t *a, long alen) {
    static int x = 100;
    assert(clen == 0);
    assert(alen == 1);

    bool stop = 0; //!(x--);

  /*
   * Insert code for deciding whether to continue running here
   */

    if (stop) {
        printf("STOP\n");
        a[0] = 1;
    }
    else
        a[0] = 0;
}

void ffiviolation(char *c, long clen, int32_t *a, long alen) {
    assert(alen == 0);

    const char* how = (const char *)c; // distinguish between normal OK and fallback

    printf("Violation... %s\n", how);

    diag_status_t new_status;    
    new_status.level = diag_status_t::ERROR;
    new_status.message = how;
    new_status.hardware_id = "ModelPlex";
    new_status.name = "VeriPhy monitor";
    diag_array_t new_diag;
    new_diag.status.push_back(new_status);
    monitor_diagnostic_pub.publish(new_diag);

    monitor_t verdict_msg;
    verdict_msg.header.stamp = current_state_timestamp > 0 ? ros::Time(current_state_timestamp) : ros::Time();
    verdict_msg.point.x = -2;
    verdict_msg.point.y = -1;
    monitor_verdict_pub.publish(verdict_msg);

    // make ffisense wait for next measurement
    last_state_timestamp = current_state_timestamp;
}

void cml_exit(void) {
    longjmp(env,1);
}

/**************************/
/* ROS-specific functions */
/**************************/

void stateCallback(const state_t::ConstPtr& msg) {
    current_state.clear();
    current_state.push_back(msg->pose.pose.position.x);
    current_state.push_back(msg->pose.pose.position.y);
    current_state.push_back(msg->pose.pose.position.z);
    current_state.push_back(tf::getYaw(msg->pose.pose.orientation));
    current_state.push_back(msg->twist.twist.linear.x);
    current_state.push_back(msg->twist.twist.linear.y);
    current_state.push_back(msg->twist.twist.angular.z);
    current_state_timestamp = msg->header.stamp.toSec();
}

void actionCallback(const action_t::ConstPtr& msg) {
    proposed_action.clear();
    proposed_action.push_back(msg->drive.speed);
    proposed_action.push_back(msg->drive.steering_angle);
}

void waypointCallback(const point_t::ConstPtr& msg) {
    current_waypoint.clear();
    current_waypoint.push_back(msg->x);  // xg
    current_waypoint.push_back(msg->y);  // yg
    current_waypoint.push_back(msg->z);  // k
}

int main (int argc, char **argv) {

    // initialize defaults
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_waypoint.push_back(10.0);
    current_waypoint.push_back(0.0);
    current_waypoint.push_back(0.0);
    proposed_action.push_back(0.0);
    proposed_action.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    current_converted_state.push_back(0.0);
    last_converted_state = current_converted_state;

    // Initialize ROS
    ros::init(argc, argv, "aa_monitor");
    ros::NodeHandle nh;
    safe_action_pub = nh.advertise<action_t>("commands/keyboard",
            QUEUE_SIZE);
    monitor_diagnostic_pub = nh.advertise<diag_array_t>("diagnostics",
            QUEUE_SIZE);
    monitor_verdict_pub = nh.advertise<monitor_t>("aa_monitor/verdict", QUEUE_SIZE);
    ros::Subscriber state_sub = nh.subscribe("ekf_localization/odom",
            QUEUE_SIZE, stateCallback);
    ros::Subscriber action_sub = nh.subscribe("aa_planner/commands",
            QUEUE_SIZE, actionCallback);
    ros::Subscriber waypoint_sub = nh.subscribe("aa_planner/waypoints",
            QUEUE_SIZE, waypointCallback);

    // Passing control to CakeML
    int sj = 0;
    sj = setjmp(env);
    /* exit after plant violation */
    /*
    if (sj == 0) {
        int ret = cml_main(argc,argv);
        printf("CML Return value: %d\n", ret);
    } else {
        printf("Abnormal exit: %d\n",sj);
    }
    */
    int ret = cml_main(argc,argv);
    printf("CML Return value: %d\n", ret);
}
