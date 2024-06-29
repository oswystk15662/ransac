import launch
import launch_ros.actions

def generate_launch_description():
    sick_node = launch_ros.actions.Node(
        package='sick_scan_xd',
        executable='sick_scan_xd',
        name='picoscan'),
    
    ransac_osw_node = launch_ros.actions.Node(
        package='ransac_osw',
        executable='ransac_osw_node',
        name='Ransac_osw'
    )

    return launch.LaunchDescription([
        sick_node, 
        ransac_osw_node
    ])