from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    with sl.group(ns='zoe'):

        sl.robot_state_publisher('zoe', 'zoe.xacro')

        sl.node('global_planning_zoe', 'spawn_robot')
        sl.node('map_simulator', 'kinematics.py', parameters = {'pub_cmd': True})

    return sl.launch_description()
