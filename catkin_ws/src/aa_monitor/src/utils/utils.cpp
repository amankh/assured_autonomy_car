/**
 * @author: edwardahn, smitsch
 */

#include <cmath>
#include <stdio.h>
#include "aa_monitor/utils/utils.h"
#include "aa_monitor/utils/consts.h"

void convertUnits(std::vector<double> &current_state, std::vector<double> &converted_state) {
    converted_state.push_back(current_state[0]*10.0); // xr [dm] @note = 0
    converted_state.push_back(current_state[1]*10.0); // yr [dm] @note = 0
    converted_state.push_back(current_state[2]*10.0); // zr [dm] @note = 0
    converted_state.push_back(current_state[3]);        // orientation radians (unused in monitor)
    converted_state.push_back(current_state[4]*10.0); // vx [dm/s]
    converted_state.push_back(current_state[5]*10.0); // vy [dm/s] @note = 0
    converted_state.push_back(current_state[6]*10.0); // vz [dm/s] @note = 0
    converted_state.push_back(current_state[7]*10.0); // xg [dm]
    converted_state.push_back(current_state[8]*10.0); // yg [dm]
    converted_state.push_back(current_state[9]*100.0);// k  [centi-(meters^-1)]
}

void convertState(std::vector<double> &current_state,
        std::vector<double> &converted_state) {
    /* see monitor_node.cpp#stateCallback */
    double xr = current_state[0];
    double yr = current_state[1];
    double zt = current_state[3];
    double xg = current_state[7];
    double yg = current_state[8];
    double k =  current_state[9];
    
    std::vector<double> coord_trafo_state;
    coord_trafo_state.push_back(0.0);
    coord_trafo_state.push_back(0.0);
    coord_trafo_state.push_back(0.0);
    coord_trafo_state.push_back(0.0);
    coord_trafo_state.push_back(current_state[4]);
    coord_trafo_state.push_back(current_state[5]);
    coord_trafo_state.push_back(current_state[6]);
    double convxg = (xg-xr)*cos(zt) + (yg-yr)*sin(zt);
    double convyg = -(xg-xr)*sin(zt) + (yg-yr)*cos(zt);
    /*     Car           Monitor
     *       x           y
     *       ^           ^
     *       |           |
     * y <---+           +---> x
     */
    coord_trafo_state.push_back(-convyg);
    coord_trafo_state.push_back(convxg);
    coord_trafo_state.push_back(-k);

    printf("Converted state xr=%f yr=%f zt=%f xg=%f yg=%f xgm=%f ygm=%f k=%f\n", xr, yr, zt, xg, yg, convxg, convyg, k);
    
    convertUnits(coord_trafo_state, converted_state);
    
    return;
}

/* Converts planner action into extCtrl as expected by monitor. */
void convertAction(std::vector<double> &proposed_action,
        std::vector<double> &converted_action) {
    
    // proposed_action see monitor_node.cpp#actionCallback
    double vset = proposed_action[0];
    double steer = proposed_action[1];
    //double vcurr = proposed_action[2];
    
    double wheelbase = 0.257;
    double d_com_rear = 0.1294;
    
    // sanity check: is steering angle roughly planned curvature
    // convert steering angle into curve radius (bicycle model)
    // simpler alternative: double r = wheelbase/tan(steer);
    double cot = steer != 0 ? 1.0/tan(steer) : 0.0;
    double r = steer != 0 ? sqrt(d_com_rear*d_com_rear + wheelbase*wheelbase*cot*cot) : 0.0;
    double ksteer = steer > 0 ? 100.0/r : steer < 0 ? -100.0/r : 0.0;
    double ksteer2 = steer != 0 ? 100.0/(wheelbase*cot) : 0.0;
    
    // actual acceleration profile to compute acceleration from vset unavailable, so we pretend to reach vset from current v within T_CYCLE_TIME
    //double tcycle = 20.0*T_CYCLE_TIME/10.0;    
    //double a = fmax(-MAX_MOTOR_ACCEL, fmin(MAX_MOTOR_ACCEL, (vset-vcurr)/tcycle)); // acceleration [dm/s^2]
    double a = proposed_action[2]; // [dm/s^2]
    double t = 0.0;      // timer expected to be reset [ds]
    double vh = proposed_action[3];   // [dm/s]    
    double vl = proposed_action[4];   // [dm/s]
    double xg = proposed_action[5];   // [dm]
    double yg = proposed_action[6];   // [dm]
    double k  = proposed_action[7];   // curvature [centi-(meters^-1)]

    printf("Converted actuate k=%f steer=%f ksteer=%f ksteer2=%f vset=%f a=%f xg=%f yg=%f vh=%f vl=%f\n", k, steer, ksteer, ksteer2, vset, a, xg, yg, vh, vl);
            
    converted_action.clear();    
    converted_action.push_back(a);
    converted_action.push_back(k);
    converted_action.push_back(t);
    converted_action.push_back(vh);
    converted_action.push_back(vl);
    converted_action.push_back(xg);
    converted_action.push_back(yg);
    
    return;
}
