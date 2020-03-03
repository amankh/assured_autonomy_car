# Assured Autonomy - Simulation

This module provides a simulation environment and training scripts for training a planner for an RWD vehicle modeled using a dynamic bicycle model with a Pacejka tire dynamic model. We use the third-party deep reinforcement learning library [rllab](https://github.com/rll/rllab) to train our policies.

Specifically, this module has training scripts to train policies that command the robot to move in straight lines or circles of a given radius. We do this in the hopes that in the future, our planners can follow paths of arbitrary curvature, conforming to the well-studied area of planning with Dubins paths.

## Code Structure

Below the main directories that form the core of this module are described.

The ```envs``` directory contains implementation of simulation environments. Each environment inherits a base environment (in ```base_env.py```), which uses a vehicle dynamics model specified in ```model.py```. An optional renderer used for viewing trained policies is also implemented in ```renderer.py```.

The ```train``` directory contains training scripts to train planners to follow circles or straight lines.

The ```scripts``` directory contains scripts that may be helpful when evaluating trained policies, exporting trained policies, etc.

## Installation Requirements

This module depends on the following:

* [rllab](https://github.com/rll/rllab)
* [cpo](https://github.com/jachiam/cpo)
* [ROS](https://www.ros.org/) (optional - see below)

Clone this simulation repository inside rllab's root directory.

Alternatively, training can be done with external simulations or directly in the real world using ROS. We use ROS Kinetic, and other versions of ROS are untested. It was  also necessary to add the following lines in place of the line 127 in the file ```rllab/algos/batch_polopt.py``` since pickling issues arise due to incompatibility between ROS and rllab:

```
params['algo'] = None
params['env'] = None
```

To restore the non-ROS version, replace the above lines with

```
params["algo"] = self
```

## Usage

Run scripts (such as any of the Python scripts in the directory ```scripts``` or ```train```) from the rllab's root directory, like so:

```
python aa_simulation/{train,scripts}/FILENAME.py
```

Arguments may need to be added to the command depending on the script. For training, the training scripts can be edited to experiment with different parameters (such as seed values, target velocity, etc.).

## Exporting Learned Policies

Each learned policy is saved in a pickle file by rllab. Unpickling this learned policy requires the user to be inside of an rllab virtual environment (ie. rllab conda environment). To allow the user to unpickle learned policies without this constraint, the command below exports the saved policy into a form that can be read without being in an rllab virtual environment.

```
python aa_simulation/scripts/export_policy PATH_TO_SAVED_POLICY
```

For example, if the policy was saved in the path ```data/exp1/params.pkl```, the user can execute the command

```
python aa_simulation/scripts/export_policy.py data/exp1/params.pkl
```

## Hints

Suppose the user trained a policy, and saved the policy in the path ```data/exp1/params.pkl```.

To view the training curves:

```
python rllab/viskit/frontend.py data/exp1
```

To evaluate the performance of learned policies in simulation:

```
python aa_simulation/scripts/eval_policy.py data/exp1/params.pkl
```

