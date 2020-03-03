#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: edwardahn

Export rllab's policy into a format that the robot can read. To do this,
load params.pkl saved by rllab, access the trained Theano model, and
save that model as a pickle file.
"""

import argparse
import joblib
import six.moves.cPickle as cPickle


def get_policy(filename):
    """
    Get saved data using joblib, and access trained model.
    """
    data = joblib.load(filename)
    policy_data = data['policy']
    policy = policy_data._f_dist
    return policy


def save_policy(policy):
    """
    Save policy as a pickle file.
    """
    f = open('model.save', 'wb')
    cPickle.dump(policy, f, protocol=cPickle.HIGHEST_PROTOCOL)
    f.close()


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=str, help='Path to snapshot file')
    args = parser.parse_args()
    return args


def main():
    args = parse_arguments()
    filename = args.file
    policy = get_policy(filename)
    save_policy(policy)
    print('Saved policy in model.save!')
    return


if __name__ == '__main__':
    main()
