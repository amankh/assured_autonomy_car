def ros_env_class(robot_type):
    """
    Return a ROS class matching the given robot type
    """
    if robot_type == 'RCCar':
        from aa_simulation.envs.ros.rccar import ROS_RCCar
        return ROS_RCCar
    elif robot_type == 'MRZR':
        from aa_simulation.envs.ros.rtk import ROS_RTK
        return ROS_RTK
    else:
        raise ValueError('Unrecognized robot type')
