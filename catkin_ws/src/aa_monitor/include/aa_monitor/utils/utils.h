#ifndef AA_MONITOR_UTILS_H
#define AA_MONITOR_UTILS_H

#include <vector>

/** \brief Convert states read from robot for monitor to read
 *
 * Convert states from (x,y,yaw,x_dot,y_dot,yaw_dot) to (t,v,xg,yg)
 * where t is the length of the sense-control-actuate loop, v is the
 * current speed, and (xg,yg) is a waypoint.
 *
 *  \param[in] current_state        Current state robot is at
 *  \param[out] converted_state     Converted form of current_state
 */
void convertState(
        std::vector<double> &current_state,
        std::vector<double> &converted_state);

/** \brief Convert actions computed by robot for monitor to read
 *
 * Convert actions from (speed,steering angle) to (a,k,t,vl,vh,xg,yg)
 * where a is acceleration, k is curvature, t is the length of the
 * sense-control-actuate loop, [vl,vh] is the range of velocities at
 * which the robot must arrive at the waypoint (xg,yg) at.
 *
 *  \param[in] proposed_action      Action that robot wants to take
 *  \param[out] converted_action    Converted form of proposed_action
 */
void convertAction(
        std::vector<double> &proposed_action,
        std::vector<double> &converted_action);

#endif /* AA_MONITOR_UTILS_H */
