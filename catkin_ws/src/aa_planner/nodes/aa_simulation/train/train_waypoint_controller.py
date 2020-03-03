#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) HRL Laboratories, LLC and Carnegie Mellon University, 2019
# Developed for DARPA and AFRL under Contract No. FA8750-18-C-0092
# Direct further inquiries to legal@hrl.com
"""
@author: Aleksey Nogin, Edward Ahn

Train a controller using TRPO or CPO so that a vehicle can navigate towards target waypoint & heading.
"""

import argparse
import json

import joblib
import lasagne.init as LI
import lasagne.layers as L
import lasagne.nonlinearities as LN
import numpy as np

import sys
import os
from subprocess import PIPE, Popen

from pkg_resources import parse_version

from rllab.algos.trpo import TRPO
from rllab.core.lasagne_layers import ParamLayer
from rllab.core.lasagne_powered import LasagnePowered
from rllab.core.network import MLP
from rllab.envs.base import Env
from rllab.misc import ext, logger
from rllab.misc.instrument import run_experiment_lite, VariantGenerator
from rllab.misc.resolve import load_class
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from sandbox.cpo.algos.safe.cpo import CPO
from sandbox.cpo.baselines.linear_feature_baseline import LinearFeatureBaseline

from aa_simulation.envs.waypoint.waypoint_env import WaypointEnv
from aa_simulation.envs.waypoint.waypoint_env_ros import WaypointEnvROS
from aa_simulation.safety_constraints.waypoint import WaypointSafetyConstraint

# Pre-trained policy and baseline
policy = None
baseline = None


def run_task(vv, log_dir=None, exp_name=None):
    global policy
    global baseline

    trpo_stepsize = 0.01
    trpo_subsample_factor = 0.2

    # Check if variant is available
    if vv['model_type'] not in ['BrushTireModel', 'LinearTireModel']:
        raise ValueError('Unrecognized model type for simulating robot')
    if vv['robot_type'] not in ['MRZR', 'RCCar']:
        raise ValueError('Unrecognized robot type')

    # Load environment
    env_class = WaypointEnvROS if vv['use_ros'] else WaypointEnv
    # Some of the parameters below are only meaninful in non-ROS environment,
    # they will simply be ignored on the ROS one.
    env = env_class(**vv)

    # Save variant information for comparison plots
    variant_file = logger.get_snapshot_dir() + '/variant.json'
    logger.log_variant(variant_file, vv)

    # Set variance for each action component separately for exploration
    # Note: We set the variance manually because we are not scaling our
    #       action space during training.
    if vv['target_speed'] is None:
        init_std_speed = max(vv['min_speed'], vv['max_speed']/20)
    else:
        init_std_speed = vv['target_speed'] / 4
    init_std_steer = 0.15
    init_std = [init_std_speed, init_std_steer]
    if vv['num_recurrent_dims']:
        init_std += [2.0] * vv['num_recurrent_dims']

    # Build policy and baseline networks
    # Note: Mean of policy network set to analytically computed values for
    #       faster training (rough estimates for RL to fine-tune).
    if policy is None or baseline is None:
        target_steering = 0.0
        target_speed = vv['target_speed']
        if target_speed is None:
            target_speed = 0.5 * (vv['min_speed'] + vv['max_speed'])
        output_mean = [target_speed, target_steering]
        if vv['num_recurrent_dims']:
            output_mean += [0.0] * vv['num_recurrent_dims']
        hidden_sizes = (32, 32)

        # In mean network, allow output b values to dominate final output
        # value by constraining the magnitude of the output W matrix. This is
        # to allow faster learning. These numbers are arbitrarily chosen.
        W_gain = min(target_speed / 5, np.pi / 15)

        mean_network = MLP(
            input_shape=(env.spec.observation_space.flat_dim,),
            output_dim=env.spec.action_space.flat_dim,
            hidden_sizes=hidden_sizes,
            hidden_nonlinearity=LN.tanh,
            output_nonlinearity=None,
            output_W_init=LI.GlorotUniform(gain=W_gain),
            output_b_init=np.array(output_mean)
        )
        policy = GaussianMLPPolicy(
            env_spec=env.spec,
            hidden_sizes=(32, 32),
            init_std=init_std,
            mean_network=mean_network
        )
        baseline = LinearFeatureBaseline(
            env_spec=env.spec,
            target_key='returns'
        )

    elif vv['reset_variance']:
        # Reset variance to re-enable exploration when using pre-trained networks
        policy._l_log_std = ParamLayer(
            policy._mean_network.input_layer,
            num_units=env.spec.action_space.flat_dim,
            param=LI.Constant(np.log(init_std)),
            name='output_log_std',
            trainable=True
        )
        obs_var = policy._mean_network.input_layer.input_var
        mean_var, log_std_var = L.get_output([policy._l_mean, policy._l_log_std])
        policy._log_std_var = log_std_var
        LasagnePowered.__init__(policy, [policy._l_mean, policy._l_log_std])
        policy._f_dist = ext.compile_function(
            inputs=[obs_var],
            outputs=[mean_var, log_std_var]
        )

    safety_baseline = LinearFeatureBaseline(
        env_spec=env.spec,
        target_key='safety_returns'
    )

    if vv['algo'] == 'TRPO':
        algo = TRPO(
            env=env,
            policy=policy,
            baseline=baseline,
            batch_size=600,
            max_path_length=env.horizon,
            n_itr=vv['n_itr'],
            discount=1.0,
            step_size=trpo_stepsize,
            plot=False,
        )
    else:
        safety_constraint = WaypointSafetyConstraint(
            baseline = safety_baseline,
            env = env,
            max_speeding_coefficient = vv['max_speeding_coeff']
        )

        algo = CPO(
            env=env,
            policy=policy,
            baseline=baseline,
            safety_constraint=safety_constraint,
            batch_size=600,
            max_path_length=env.horizon,
            n_itr=vv['n_itr'],
            discount=1.0,
            step_size=trpo_stepsize,
            gae_lambda=0.95,
            safety_gae_lambda=1,
            optimizer_args={'subsample_factor': trpo_subsample_factor},
            plot=False
        )
    algo.train()


def parse_arguments():
    nproc = int(os.popen('nproc').read().rstrip())
    parser = argparse.ArgumentParser()
    parser.add_argument('--algo', choices=['trpo', 'cpo'],
            default='trpo', help='Type of algorithm to use to train agent (default: trpo)')
    parser.add_argument('--network', type=str,
            help='Path to snapshot file of pre-trained network')
    parser.add_argument('--reuse_meta_params', type=str,
            help='Path to variant.json to reuse (note: only some meta-params are reused - the focus is on making sure we do not do any further parameter sweep)')
    parser.add_argument('--n_itr', type=int, default=600,
            help='Number of iterations to perform in each experiment (default: 600)')
    parser.add_argument('--ros', dest='ros', action='store_true',
            help='Use the ROS interface')
    parser.add_argument('--no-ros', dest='ros', action='store_false',
            help='Use the built-in simulator (default)')
    parser.set_defaults(ros=False)
    parser.add_argument('--target_speed', type=float, default=None,
            help='Speed the agent should be trained to maintain (default is to use variable speed between min_speed and max_speed)')
    parser.add_argument('--reset-variance', dest='reset_variance', action='store_true',
            help='Reset variance of pre-trained networks to re-expand exploration (default)')
    parser.add_argument('--no-reset-variance', dest='reset_variance', action='store_false',
            help='Do not reset variance of pre-trained networks')
    parser.set_defaults(reset_variance=True)
    parser.add_argument('--min_speed', type=float, default=0.5,
            help='Minimal speed the agent should be trained to maintain')
    parser.add_argument('--max_speed', type=float, default=6.0,
            help='Maximal speed the agent should be trained to maintain')
    parser.add_argument('--target_heading', dest='target_heading', action='store_true',
            help='Train the agent to target waypoint + heading (default)')
    parser.add_argument('--target_curvature', dest='target_heading', action='store_false',
            help='Train the agent to target waypoint + curvature')
    parser.add_argument('--robot_type', type=str, default='MRZR', choices=['MRZR', 'RCCar'],
            help='Robot model to use (default: MZRZ)')
    parser.add_argument('--num_parallel', type=int, default=nproc,
            help='Number of parallel threads to use for running the vehicle model (default: %i, ignored in ROS mode)'%(nproc,))
    parser.set_defaults(target_heading=True)
    args = parser.parse_args()
    return args


def main():
    global policy
    global baseline

    # Load pre-trained network if available
    args = parse_arguments()
    if args.network is not None:
        data = joblib.load(args.network)
        policy = data['policy']
        baseline = data['baseline']
        args.network = os.path.abspath(os.path.realpath(args.network))

    variant = None
    if args.reuse_meta_params is not None:
        with open(args.reuse_meta_params, 'r') as f:
            variant = json.load(f)

    os.chdir(os.path.dirname(sys.argv[0]))
    os.chdir('../..')
    if args.network is not None:
        args.network = os.path.relpath(args.network)

    p = Popen("python -V", shell=True, stdout=PIPE, stderr=PIPE)
    (out, err) = p.communicate()
    p.wait()
    if out and not err:
        # Unsure why these things differ on different versions
        err = out
    if parse_version(err.strip().split()[1].decode("utf-8")) < parse_version('3.3.0'):
        os.environ["PATH"] = "/usr/local/defpython3:" + os.environ["PATH"]

    # Run multiple experiment variants at once
    vg = VariantGenerator()
    vg.add('seed', [100,200,300])

    # Non-configurable parameters (do not change)
    vg.add('training script', [sys.argv[0]])
    vg.add('git revision', [os.popen('cd aa_simulation; git rev-parse HEAD').read().rstrip()])
    vg.add('git status', [os.popen('cd aa_simulation; git status -s').read().rstrip()])
    vg.add('use_ros', [args.ros])
    if args.algo == 'trpo':
        vg.add('algo', ['TRPO'])
    else:
        vg.add('algo', ['CPO'])
        if variant and 'max_speeding_coeff' in variant:
            vg.add('max_speeding_coeff',[variant['max_speeding_coeff']])
        else:
            vg.add('max_speeding_coeff',[1.05,1.1])

    # Configurable parameters
    #   Options for model_type: 'BrushTireModel', 'LinearTireModel'
    #   Options for robot_type: 'MRZR', 'RCCar'
    if variant:
        target_heading = variant.get('target_heading', True)
    else:
        target_heading = args.target_heading
    vg.add('target_heading', [target_heading])
    def add(tag, defvalue):
        if variant and tag in variant:
            vg.add(tag, [variant[tag]])
        else:
            vg.add(tag, defvalue)
    add ('precision', [0.6000001 if target_heading else 0.5]) # 0.6 is just enough to allow dt=0.025 with max speed of 6m/s
    if args.ros:
        vg.add('dt', [None])
    else:
        add ('dt', [0.02 if args.robot_type == 'MRZR' else 0.035])
    add('success_reward', [50000.0, 30000.0, 15000.0])
    add('lambda1', [75,50,40])
    add('model_type', ['BrushTireModel'])
    add('robot_type', [args.robot_type])
    add('mu_s', [1.37])
    add('mu_k', [1.96])
    add('path_width', [0.1])
    add('wrong_turn_penalty', [125.0,75.0,200.0])
    #add('num_recurrent_dims', [2,1,0])
    add('num_recurrent_dims', [0])

    # These are not taken from the variant, even when variant is there
    if args.network is not None:
        vg.add('reset_variance', [args.reset_variance])
    vg.add('n_itr', [args.n_itr])
    vg.add('pretrained', [args.network])
    vg.add('target_speed', [args.target_speed])
    vg.add('min_speed', [args.min_speed])
    vg.add('max_speed', [args.max_speed])
    vg.add('num_parallel', [args.num_parallel])

    print('Number of Configurations: ', len(vg.variants()))

    # Run each experiment variant
    for vv in vg.variants():
        run_experiment_lite(
            stub_method_call=run_task,
            variant=vv,
            n_parallel=args.num_parallel,
            snapshot_mode='last',
            seed=vv['seed']
        )


if __name__ == '__main__':
    main()
