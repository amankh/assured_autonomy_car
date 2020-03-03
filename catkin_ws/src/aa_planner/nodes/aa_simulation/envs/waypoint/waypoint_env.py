#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) HRL Laboratories, LLC and Carnegie Mellon University, 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Aleksey Nogin, Edward Ahn

Environment for training a local planner to move towards target waypoint & heading.
"""

import numpy as np
import math

from rllab.spaces import Box
from rllab.misc import logger

from aa_simulation.envs.waypoint.heading import WaypointHeadingCalc
from aa_simulation.envs.waypoint.curvature import WaypointCurvatureCalc
from aa_simulation.envs.base_env import VehicleEnv
from aa_simulation.misc.utils import normalize_angle, rotate_state, translate_state

class WaypointEnv(VehicleEnv):
    """
    Simulation environment for an RC car moving towards a waypoint (WP) with a
    desired heading at WP. The reward function encourages the agent to get the
    two points "rmin to the left of the vehicle" and "rmin to the right of the
    vehicle" as close as possible to the two points "rmin to the left of the
    WP" and "rmin to the right of the WP" respectively, where "rmin" is the
    minimum turning radius, and "to the left"/"to the right" of vehicle/WP
    means direction -90deg/90deg from the vehicle heading / desired target
    heading respectively.

    The environment uses the WP-centric coordinates - the WP is at (0,0) and
    the desired heading is (1,0) - "going right" along the x axis (this is
    the same as the line-following controller, but different from the Keymaera
    model that uses (0,1) instead).
    """

    # Which eval_policy plots to show; overrides base class defaults to show all
    show_distance_plot = False
    speed_plot_name = 'Speed Error'
    show_distance_distibution_plot = False

    # When drawing waypoints, how long is the arrow (in multiples of precision)
    RENDER_WP_ARROW_LENGTH_COEFF = 2.5
    RENDER_WP_CIRCLE_RADIUS_COEFF = 0.4
    RENDER_WP_ARROW_HEAD_WIDTH_COEFF = 0.6

    def __init__(self,
            min_speed = None, max_speed = None,
            target_speed = None,
            precision=None, dt=0.02,
            lambda1=10, success_reward=30000.0,
            robot_type='MRZR',
            target_heading=True,
            run_sim_idle_steps=20,
            path_width=0.1,
            wrong_turn_penalty=100.0,
            **kwarg):
        """
        Initialize super class parameters, and waypoint precision.
        """
        super(WaypointEnv, self).__init__(robot_type = robot_type, dt = dt, **kwarg)

        # Vehicle parameters
        self._max_v = self._params['max_v']
        self._variable_speed = (target_speed is None)
        err = None
        if self._variable_speed:
            if min_speed is None or max_speed is None:
                err = 'Either target_speed, or both min_speed and max_speed need to be specified'
            else:
                self._min_speed = min_speed
                self._max_speed = max_speed
                self._target_speed = 0.5 * (min_speed+max_speed) # Placeholder until get_initial_state is called
                if self._max_speed > self._max_v:
                    err = 'Attempted to set maximal target velocity (%.2f) greater that max velocity (%.2f) for the \'%s\' robot type' % (self._max_speed, self._max_v, self.robot_type)
        else:
            if min_speed is not None or max_speed is not None:
                err = 'Only one of target_speed, or min_speed & max_speed should to be specified'
            elif target_speed > self._max_v:
                err = 'Attempted to set target velocity (%.2f) greater that max velocity (%.2f) for the \'%s\' robot type' % (target_speed, self._max_v, self.robot_type)
            else:
                self._target_speed = target_speed
        if err is not None:
            print(err)
            raise ValueError(err)
        if precision is None:
            precision = 4.0 * self._max_v * self._dt
        elif precision < 4.0 * self._max_v * self._dt:
            err = 'Attempted to set precision (%.2f) less than minimal allowed value (4*max_v*dt = %.2f) for the \'%s\' robot type' % (precision, 4.0 * self._max_v * self._dt, self.robot_type)
            print(err)
            raise ValueError(err)
        self._precision = precision
        self._rmin = self._params['r_min'] # Minimal turning radius
        self._max_wp_dist = self._params['max_wp_dist'] # Max distance to the next WP

        # Run sim to get a meaningful initial condition
        self._run_sim_idle_steps = run_sim_idle_steps

        # Reward function parameters
        self._lambda1 = lambda1
        self._success_reward = success_reward
        self._path_width = path_width # Do not allow "wrong" turns when we are this much to the side of the x axis
        self._wrong_turn_penalty = wrong_turn_penalty

        # Give ourselves enough time to reach the WP.
        if self._variable_speed:
            self._horizon_length = self._max_wp_dist / (self._min_speed * self._dt)
        else:
            # 2.5 is an ad-hoc parameter
            self._horizon_length = 2.5 * self._max_wp_dist / (self._target_speed * self._dt)

        # Waypoint Calculations
        self._target_heading = target_heading
        if target_heading:
            self._calc = WaypointHeadingCalc(self._max_v, self._precision, self._dt, self._rmin)
        else:
            self._calc = WaypointCurvatureCalc(self._max_v, self._precision, self._dt, self._rmin)

        # State used when rendering a simulated run from eval_policy.py
        self._render_rotate = 0
        self._render_translate = np.zeros(2)
        self._prev_actions = None
        self._wps = None
        if not self._target_heading:
            self._far_arc = None
            self._near_arc = None

        # Collect additional data for eval_policy
        self._collected_successes = []
        self._collected_failures = []
        self._collected_jerks = []
        self._collected_sjerks = []

        self._collected_max_speeding = 0.0
        self._collected_max_speeding_speed_cmd = None
        self._collected_max_speeding_speed = None
        self._collected_max_speeding_target_speed = None
        self._collected_max_speeding_init_speed = None
        self._collected_speed_violations = []

        self._collected_max_yaw_dot = np.zeros(1)
        self._collected_max_velocity_angle = np.zeros(1)
        self._collected_max_yd_diff = np.zeros(1)

        self._collected_max_steer_away = 0.0
        self._collected_max_steer_away_y = None
        self._collected_max_steer_away_yaw = None
        self._collected_max_steer_away_cmd = None
        self._collected_steer_violations = []
 

    def get_random_waypoint(self):
        # 0.25 is an ad-hoc parameter
        max_y = 0.25 * self._max_wp_dist

        x = np.random.uniform(-self._max_wp_dist, 0)
        y = np.random.uniform(-max_y, max_y)
        yaw = np.random.uniform(-np.pi/3, np.pi/3)
        speed = np.random.uniform(0, self._max_v)
        if self.robot_type == 'RCCar':
            velocity_angle = np.random.uniform(-np.pi/6, np.pi/6)
            yaw_dot = np.random.uniform(-2.0, 2.0)
        elif self.robot_type == 'MRZR':
            velocity_angle = np.random.uniform(-0.32, 0.32)
            # Assume limited skidding!
            # First, compute yaw_dot in absence of skidding
            curvature = np.sin(velocity_angle) / self._params['L_r'] 
            yaw_dot = curvature * speed
            # Then add an error term to it
            yaw_dot += np.random.uniform(-0.15, 0.15)
        else:
            raise ValueError('Unrecognized robot type: ' + self.robot_type)

        x_dot = speed * np.cos(velocity_angle)
        y_dot = speed * np.sin(velocity_angle)

        state = np.array([x, y, yaw, x_dot, y_dot, yaw_dot])

        max_curv = 1.0 / self._rmin
        curvature = np.random.uniform(-max_curv, max_curv)
        if self._variable_speed:
            speed = np.random.uniform(self._min_speed, self._max_speed)
            speed_cmd = np.random.uniform(self._min_speed, self._max_v)
        else:
            speed = self._target_speed
            speed_cmd = np.random.uniform(0, self._max_v)

        self._calc.set_waypoint(curvature, max_speed = speed)

        curv_cmd = np.random.uniform(-max_curv, max_curv)
        action = np.array([speed_cmd, curv_cmd])
        if self.HAS_KAPPA:
            # Try to make the x_dot/y_dot/yaw_dot combination more realistic
            for i in range(self._run_sim_idle_steps):
                state = self._model.state_transition(state, action, self._dt)
            state[0] = x
            state[1] = y
            state[2] = yaw

        return state, speed, curvature

    @property
    def get_initial_state(self):
        """
        Get initial state of car when simulation is reset.
        """
        self._num_steps = 0

        def_action = np.array([self._target_speed, 0])
        if self._simulation_num_paths is None:
            # Randomly initialize state for better learning
            state, self._target_speed, self._target_curvature = self.get_random_waypoint()
            self._prev_actions = np.array([def_action, def_action])
        else:
            if self._prev_actions is None:
                self._prev_actions = np.array([def_action, def_action])
            if not self._wps:
                # We prerandomize all the WPs upfront so that they stay the same, no matter what policy is used
                self._wps = []
                num_wps = self._simulation_num_paths
                # XXX: HACK: a few more for ROS environments, where waypoints may turn to be infeasible
                # because of the actual vehicle state
                if not self.HAS_KAPPA:
                    num_wps += 1 + int(num_wps/10)
                for i in range(num_wps):
                    # For evaluation, only use feasible waypoints that are not reached already
                    state_type = 1
                    while state_type != 0:
                        waypoint = self.get_random_waypoint()
                        state_type = self._calc.get_state_type(waypoint[0])[0]
                    self._wps.append(waypoint)
            state, self._target_speed, self._target_curvature = self._wps.pop(0)

            if self._subplot:
                state = self.render_next_wp(state)

        self._calc.set_waypoint(max_speed = self._target_speed, curvature = self._target_curvature)

        if self.num_recurrent_dims:
            state = np.append(state, [0.0] * self.num_recurrent_dims)
        return state

    def render_next_wp(self, state):
        # We compute the last waypoint w.r.t the old on (not vehicle position)
        x, y, yaw, x_dot, y_dot = state[0:5]
        if self._state is None:
            self._state = np.array(state)
            # starting position on screen: 0,0; facing right
            self._state[0] = 0
            self._state[1] = 0
            self._state[2] = 0
        else:
            #Use velocity direction as heading; needs to be computed the same way as in WaypointCalc.get_state_type
            speed = math.sqrt(x_dot**2 + y_dot**2)
            if speed > 0.05:
                 yaw += math.atan2(y_dot, x_dot)
        # In prev wp coordinates, prev wp was (0,0.0); in new wp ones, old wp was (x,y,yaw)
        #     compute: new = wp_translate(wp_rotate(old))
        wp_rotate = yaw
        wp_translate = np.array([x,y])

        # new translation: render = render_translate(render_rotate(-wp_rotate(-wp_translate(new))
        self._render_rotate -= wp_rotate
        translation = translate_state(rotate_state(-state, self._render_rotate), self._render_translate)
        self._render_translate = translation[:2]
        # Translate from old wp coords to new wp coords
        state = translate_state(rotate_state(self._state, wp_rotate), wp_translate)

        self.render_waypoint()

        return state

    def get_reward(self, oldstate, action, state):
        """
        Reward function definition.
        """
        #state_type, distance, speed = self._calc.get_state_type(state, want_print = True)
        old_state_type, old_distance, old_speed, _ = self._calc.get_state_type(oldstate)
        new_state_type, new_distance, new_speed, _ = self._calc.get_state_type(state)

        done = (state[0] >= 0)

        if done:
            # Ignore the new state, it's one past what we care about
            distance = old_distance
            speed = old_speed
            state_type = old_state_type
        else:
            distance = new_distance
            speed = new_speed
            state_type = new_state_type

        reward = - distance
        if speed > self._target_speed:
            # speeding penalty. We are OK multiplying it in the last step, as speed at waypoing is particularly omportant
            # We do not explicitly penalize for low speed - the slower distance reduction will do it for us
            reward -= self._lambda1 * (speed - self._target_speed)
        if done:
            # Provide the same reward for all remaining steps we are not goint to be simulating. We assume no discounting (discount==1.0)
            reward *= self._horizon_length - self._num_steps
            if state_type == 1:
                reward += self._success_reward
            if self._simulation_num_paths is not None:
                color = 'green' if state_type == 1 else 'red'
                self.render_waypoint(color)

        oldyaw = normalize_angle(oldstate[2])
        if ((oldstate[1] > self._path_width and oldyaw > 0 and action[1] > 0)
                or (oldstate[1] < - self._path_width and oldyaw < 0 and action[1] < 0)):
            # This one should not get multiplied when in the last step
            reward -= self._wrong_turn_penalty

        self._num_steps += 1

        info = {
            'done' : done,
            'dist': distance,
            'state_type_before': old_state_type,
            'state_type': state_type,
            'vel': (speed - self._target_speed),
            'jerk': action[0] - 2 * self._prev_actions[0][0] + self._prev_actions[1][0],
            'sjerk': action[1] - 2 * self._prev_actions[0][1] + self._prev_actions[1][1],
            'yaw' : oldyaw
            }
        self._prev_actions[1] = self._prev_actions[0]
        self._prev_actions[0] = action[0:2]

        return reward, info


    @property
    def observation_min(self):
        if self._target_heading:
            o_min = [-self._max_wp_dist, -self._max_wp_dist, -np.pi]
        else:
            o_min = [-math.sqrt(2.0)*self._max_wp_dist, -math.sqrt(2.0)*self._max_wp_dist, -1.0 / self._rmin]
        o_min += [0.0, -np.pi, -np.inf]
        if self._variable_speed:
            o_min += [self._min_speed]
        return o_min

    @property
    def observation_max(self):
        if self._target_heading:
            o_max = [self._max_speed * self._dt, self._max_wp_dist, np.pi]
        else:
            o_max = [math.sqrt(2.0)*self._max_wp_dist, math.sqrt(2.0)*self._max_wp_dist, 1.0 / self._rmin]
        o_max += [0.0, np.pi, np.inf]
        if self._variable_speed:
            o_max += [self._max_speed]
        return o_max

    def state_to_observation(self, state):
        """
        Prepare state to be read as input to neural network.
        """
        if self._target_heading:
            obs = [
                state[0], #x
                state[1], #y
                normalize_angle(state[2]) # yaw
            ]
        else:
            x = state[0]
            y = state[1]
            yaw = state[2]
            obs = [
                x * math.sin(yaw) - y * math.cos(yaw), #xg
                -x * math.cos(yaw) - y * math.sin(yaw), #yg
                - self._target_curvature #negated for historical reasons
            ]
        obs += [
            math.sqrt(state[3]**2 + state[4]**2), #speed
            math.atan2(state[4], state[3]), #Relative angle of velocity
            state[5]] #yaw_dot
        if self._variable_speed:
            obs.append(self._target_speed)
        if self.num_recurrent_dims:
            obs += list(state[-self.num_recurrent_dims:])
        return np.array(obs)

    def log_diagnostics(self, paths):
        """
        Log extra information per iteration based on collected paths.
        """
        dists = []
        vels = []
        if self.HAS_KAPPA:
            kappas = []
        jerks = []
        sjerks = []
        durations = []
        successes = 0
        for path in paths:
            durations.append(len(path['env_infos']['dist']))
            dists.append(np.min(path['env_infos']['dist']))
            vels.append(np.mean(path['env_infos']['vel']))
            if self.HAS_KAPPA:
                kappas.append(np.mean(path['env_infos']['kappa']))
            jerks.append(np.mean(np.square(path['env_infos']['jerk'])))
            sjerks.append(np.mean(np.square(path['env_infos']['sjerk'])))
            if path['env_infos']['state_type'][-1] == 1:
                successes += 1
        dists = np.abs(dists)
        vels = np.abs(vels)
        if self.HAS_KAPPA:
            kappas = np.abs(kappas)

        logger.record_tabular('AvgDurationFrac', np.mean(durations)/float(self._horizon_length))
        logger.record_tabular('MinDurationFrac', np.min(durations)/float(self._horizon_length))
        logger.record_tabular('MaxDurationFrac', np.max(durations)/float(self._horizon_length))
        logger.record_tabular('FracSuccess', successes/float(len(paths)))
        logger.record_tabular('LogAvgMinDistance', np.log(np.mean(dists)))
        logger.record_tabular('AverageAbsVelocityErr', np.mean(vels))
        logger.record_tabular('MaxAbsVelocityErr', np.max(vels))
        if self.HAS_KAPPA:
            logger.record_tabular('AverageKappa', np.mean(kappas))
            logger.record_tabular('MaxKappa', np.max(kappas))
        logger.record_tabular('AverageSqLongitJerk', np.mean(jerks))
        logger.record_tabular('AverageSqSteerJerk', np.mean(sjerks))
        logger.record_tabular('MaxAvgSqLongitJerk', np.max(jerks))
        logger.record_tabular('MaxAvgSqSteerJerk', np.max(sjerks))

    def reset_renderer(self):
        # We want a continuous drive
        pass

    def configure_renderer(self, fig, subplot):
        super(WaypointEnv, self).configure_renderer(fig, subplot)
        assert(self._wps is not None) # First call to get_initial_state is supposed to come earlier
        xs=[0]
        ys=[0]
        rotate=0
        translate=np.zeros(2)
        wps = self._wps.copy()
        wps.insert(0, (self._state, self._target_speed, self._target_curvature))
        for (wp, _target_speed, curvature) in wps:
            # This should be the same as the similar computation in get_initial_state
            # XXX: TODO: have a single helper function for both?
            x, y, yaw, x_dot, y_dot, _, = wp
            speed = math.sqrt(x_dot**2 + y_dot**2)
            if speed > 0.05:
                 yaw += math.atan2(y_dot, x_dot)
            wp_rotate = yaw
            wp_translate = np.array([x,y])
            rotate -= wp_rotate
            translation = translate_state(rotate_state(-wp, rotate), translate)
            translate = translation[:2]
            # The following computation is the same as in render_waypoint
            wp = np.zeros(6)
            origin = translate_state(rotate_state(wp, rotate), translate)
            xs.append(origin[0])
            ys.append(origin[1])
            if self._target_heading:
                wp[0] = self.RENDER_WP_ARROW_LENGTH_COEFF * self._precision
                arrow_head = translate_state(rotate_state(wp, rotate), translate)
                xs.append(arrow_head[0])
                ys.append(arrow_head[1])
        extra = max(self._params['L_f'], self._params['L_r']) + 0.5 * self._params['wheel_dia']

        subplot.set_xlim(np.min(xs)-extra,np.max(xs)+extra)
        subplot.set_ylim(np.min(ys)-extra,np.max(ys)+extra)
        self.render_next_wp(self._state)

    def render_waypoint(self, color='purple'):
        if self._subplot:
            import matplotlib.patches as patches
            wp = np.zeros(6)
            origin = translate_state(rotate_state(wp, self._render_rotate), self._render_translate)
            circ = patches.Circle((origin[0], origin[1]), self.RENDER_WP_CIRCLE_RADIUS_COEFF * self._precision, color=color, fill = True)
            self._subplot.add_patch(circ)
            if self._target_heading:
                wp[0] = self.RENDER_WP_ARROW_LENGTH_COEFF * self._precision # Arrow length
                arrow_dir = translate_state(rotate_state(wp, self._render_rotate), self._render_translate) - origin
                self._subplot.arrow(origin[0], origin[1], arrow_dir[0], arrow_dir[1], color=color, length_includes_head=True, head_width = self.RENDER_WP_ARROW_HEAD_WIDTH_COEFF * self._precision)

    def get_render_state(self):
        render_state = translate_state(rotate_state(self._state, self._render_rotate), self._render_translate)
        return render_state

    def render(self):
        if self._renderer == None:
            self.initialize_renderer()
        if not self._target_heading:
            # Targeting curvature raw the arcs for the legal waypoint location
            x, y, yaw, _ , _, _ = self._state
            import matplotlib.patches as patches
            if self._far_arc is not None:
                self._far_arc.remove()
            if self._near_arc is not None:
                self._near_arc.remove()
            #XXX:TODO:support curvature = 0
            #turn center
            y_c_wpcoord = y + math.cos(yaw) / self._target_curvature
            x_c_wpcoord = x - math.sin(yaw) / self._target_curvature

            x_c, y_c, _, _, _, _ = translate_state(rotate_state([x_c_wpcoord, y_c_wpcoord, 0,0,0,0], self._render_rotate), self._render_translate)

            diam_near = 2.0/ self._target_curvature - 2.0*self._precision
            diam_far = 2.0/ self._target_curvature + 2.0*self._precision
            t1 = math.atan2(-y_c_wpcoord, -x_c_wpcoord) # angle of WP from center
            if self._target_curvature <= 0:
                t1 -= math.pi
                t2 = yaw - math.pi * 0.5 # angle of car from center
            else:
                t2 = t1
                t1 = yaw - math.pi * 0.5 # angle of car from center
            t1 += self._render_rotate
            t2 += self._render_rotate
            t1 *= 180.0 / math.pi
            t2 *= 180.0 / math.pi
            if t1 < 0:
               t1 += 360.0
            if t2 < 0:
               t2 += 360.0
            car_render = translate_state(rotate_state(self._state, self._render_rotate), self._render_translate)
            print("car@(wp coord:%.3f,%.3f %.3f, render coord:%.3f,%.3f %.3f), curv=%.3f (turn rad = %.3f), center@(wp coord:%.3f,%.3f, render coord:%.3f,%.3f), arc=%.3f-->%.3f" % (x,y,yaw,car_render[0],car_render[1],car_render[2],self._target_curvature,1.0/self._target_curvature, x_c_wpcoord,y_c_wpcoord,x_c,y_c,t1,t2))
            self._near_arc = patches.Arc((x_c,y_c), diam_near, diam_near, theta1 = t1, theta2 = t2, color="purple", ls="--")
            self._far_arc = patches.Arc((x_c,y_c), diam_far, diam_far, theta1 = t1, theta2 = t2, color="purple", ls = "--")
            self._subplot.add_patch(self._near_arc)
            self._subplot.add_patch(self._far_arc)
        # This should go last as it calls "update", records a movie, etc
        super(WaypointEnv, self).render()

    def analyze_rollout(self, path, skip=0):
        super(WaypointEnv, self).analyze_rollout(path, skip)
        dist = path['env_infos']['dist'][-1]
        state_type = path['env_infos']['state_type'][-1]
        (self._collected_successes if state_type == 1 else self._collected_failures).append(dist)
        self._collected_jerks.append(np.mean(np.square(path['env_infos']['jerk'])))
        self._collected_sjerks.append(np.mean(np.square(path['env_infos']['sjerk'])))
        observations = path['observations']
        speed = observations[:, 3]
        actions = path['actions']
        initial_speed = speed[0]
        target = self._target_speed
        if self._variable_speed:
            target = observations[0,6]
        max_speed = max(target, initial_speed)
        self._collected_speed_violations.append(np.clip(np.max(speed)/max_speed - 1.05, 0, np.inf))
        worst = np.argmax(speed)
        if speed[worst] / max_speed > self._collected_max_speeding:
            self._collected_max_speeding = speed[worst] / max_speed
            self._collected_max_speeding_speed = speed[worst]
            self._collected_max_speeding_speed_cmd = actions[worst, 0]
            self._collected_max_speeding_speed_init_speed = initial_speed
            self._collected_max_speeding_speed_target_speed = target

        y = observations[:, 1]
        yaw = path['env_infos']['yaw']
        steer = actions[:, 1]
        driving_away = steer * (np.sign(y) + np.sign(yaw))
        scale = np.abs(y * yaw * steer) * (driving_away > 0)
        worst = np.argmax(scale)
        if scale[worst] > self._collected_max_steer_away:
            self._collected_max_steer_away = scale[worst]
            self._collected_max_steer_away_y = y[worst]
            self._collected_max_steer_away_yaw = yaw[worst]
            self._collected_max_steer_away_cmd = steer[worst]
        self._collected_steer_violations.append(scale)
        yaw_dot = observations[:,5]
        velocity_angle = observations[:,4]
        curvature = np.sin(velocity_angle) / self._params['L_r']
        computed_yaw_dot = curvature * speed
        yd_diff = computed_yaw_dot - yaw_dot
        lendiff = len(yaw_dot) - len(self._collected_max_yaw_dot)
        if lendiff > 0:
            self._collected_max_yaw_dot = np.pad(self._collected_max_yaw_dot, (0,lendiff), mode='constant')
            self._collected_max_velocity_angle = np.pad(self._collected_max_velocity_angle, (0,lendiff), mode='constant')
            self._collected_max_yd_diff = np.pad(self._collected_max_yd_diff, (0,lendiff), mode='constant')
        elif lendiff < 0:
            yaw_dot = np.pad(yaw_dot, (0, -lendiff), mode='constant')
            velocity_angle = np.pad(velocity_angle, (0, -lendiff), mode='constant')
            yd_diff = np.pad(yd_diff, (0, -lendiff), mode='constant')
        self._collected_max_yaw_dot = np.maximum(self._collected_max_yaw_dot, np.abs(yaw_dot))
        self._collected_max_velocity_angle = np.maximum(self._collected_max_velocity_angle, np.abs(velocity_angle))
        self._collected_max_yd_diff = np.maximum(self._collected_max_yd_diff, np.abs(yd_diff))

    def get_plots(self):
        (curves, distributions) = super(WaypointEnv, self).get_plots()
        if self._collected_failures:
            distributions.append((np.array(self._collected_failures), 'Failure distance', 'm'))
        if self._collected_successes:
            distributions.append((np.array(self._collected_successes), 'Success distance', 'm'))
        curves.append((self._collected_max_yaw_dot, 'Max yaw dot over multiple runs', 'rad/s'))
        curves.append((self._collected_max_velocity_angle, 'Max velocity angle over multiple runs', 'rad'))
        curves.append((self._collected_max_yd_diff, 'Max |yaw_dot-yaw_dot(velocity)| over multiple runs', 'rad/s'))
        return (curves, distributions)

    def print_means(self):
        super(WaypointEnv, self).print_means()
        vels = np.abs(np.concatenate(self._collected_vels, axis=None))
        print('\tMean Abs Speed Error:\t%.5f +/- %.5f'
            % (vels.mean(), vels.std()))
        print("\tSuccess Percentage:\t%.2f%%"
            % (len(self._collected_successes) * 100.0 /
                (len(self._collected_successes) + len(self._collected_failures)),))
        if self._collected_successes:
            successes = np.sqrt(np.array(self._collected_successes) * 0.5)
            print('\tMean Success Distance:\t%.5f +/- %.5f'
                % (successes.mean(), successes.std()))
        if self._collected_failures:
            failures = np.sqrt(np.array(self._collected_failures) * 0.5)
            print('\tMean Failure Distance:\t%.5f +/- %.5f'
                % (failures.mean(), failures.std()))
        sjerks = np.concatenate(self._collected_sjerks, axis=None)
        print('\tMean Steer Cmd Jerk:\t%.5f +/- %.5f'
            % (sjerks.mean(), sjerks.std()))
        jerks = np.concatenate(self._collected_jerks, axis=None)
        print('\tMean Speed Cmd Jerk:\t%.5f +/- %.5f'
            % (jerks.mean(), jerks.std()))
        print('\tMax Speeding Coefficient:\t%.5f (init=%.3f, targ=%.3f, cmd=%.3f -> %.3f)' % (self._collected_max_speeding, self._collected_max_speeding_speed_init_speed, self._collected_max_speeding_speed_target_speed, self._collected_max_speeding_speed_cmd, self._collected_max_speeding_speed))
        speedings = np.concatenate(self._collected_speed_violations, axis=None)
        print('\tMean Speeding Violation Frac:\t%.5f +/- %.5f' % (speedings.mean(), speedings.std()))
        if self._collected_max_steer_away_y is None:
            print('\tMax Incorrect Steering:\t0.0')
        else:
            print('\tMax Incorrect Steering:\t%.6f (y=%.3f, yaw=%.3f -> steer=%.3f)' % (self._collected_max_steer_away, self._collected_max_steer_away_y, self._collected_max_steer_away_yaw, self._collected_max_steer_away_cmd))
        missteerings = np.concatenate(self._collected_steer_violations, axis=None)
        print('\tMean Steering Violation:\t%.8f +/- %.8f' % (missteerings.mean(), missteerings.std()))
