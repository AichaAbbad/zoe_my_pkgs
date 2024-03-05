from launch.actions import (EmitEvent, LogInfo)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, LocalSubstitution)
from simple_launch import SimpleLauncher
from simple_launch.events import When, OnProcessExit, OnProcessStart, OnShutdown, OnExecutionComplete, OnProcessIO

def generate_launch_description():
    
    sl = SimpleLauncher()

    sl.declare_arg('my_code', 'true', description = 'Whether to use my code')
    sl.declare_arg('verbosity', 'reqres')
    
    sl.include('map_simulator', 'simulation2d_launch.py',
            launch_arguments={'map': sl.find('map_simulator', 'map.yaml'),
                              'map_server': True})

    # also run RViz
    sl.rviz(sl.find('global_planning_zoe_interface', 'r2d2.rviz'))


    sim = sl.node('global_planning_zoe', 'main_test', name ='sim', output='screen')
    with sl.group(when = When(sim, OnProcessStart)):
      node = sl.call_service('IntPose',{'pose':1.0}, verbosity = sl.arg('verbosity'))
      
    # here m supposed to get my values
    x = 0.0 
    y = 0.0

    #and spawn a slider controller
    #sl.node('slider_publisher', arguments = [sl.find('map_simulator', 'cmd_vel_multi.yaml')])
    
    for name, color in (('r2d2', [255,0,0]),
                        #('r2d3', [0,255,0]),
                        #('r2d4', [0,0,255])
                      ):
    
        sl.include('map_simulator', 'single_launch.py',
                  launch_arguments={'name': name, 'color': str(color), 'x': x, 'y': y})

    
    return sl.launch_description()
