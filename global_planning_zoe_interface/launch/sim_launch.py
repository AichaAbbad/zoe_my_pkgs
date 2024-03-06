from launch.actions import (EmitEvent, LogInfo)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, LocalSubstitution)
from simple_launch import SimpleLauncher
from simple_launch.events import When, OnProcessExit, OnProcessStart, OnShutdown, OnExecutionComplete, OnProcessIO

def generate_launch_description():
    
    sl = SimpleLauncher()
  
    sl.include('map_simulator', 'simulation2d_launch.py',
            launch_arguments={'map': sl.find('global_planning_zoe_interface', 'map.yaml'),
                              'map_server': True})

    # also run RViz
    sl.rviz(sl.find('global_planning_zoe_interface', 'my_zoe.rviz'))

    sl.node('global_planning_zoe', 'main_test', output='screen')

    sl.include('global_planning_zoe_interface', 'spawn_launch.py')

    #and spawn a slider controller
    #sl.node('slider_publisher', arguments = [sl.find('map_simulator', 'cmd_vel_multi.yaml')])

    return sl.launch_description()
