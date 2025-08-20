from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def generate_launch_description():
    # Declare the launch argument 'number_of_agents'
    number_of_agents_arg = DeclareLaunchArgument(
        'number_of_agents',
        default_value='3',
        description='Number of pbtm_node instances to launch'
    )

    # Get the number_of_agents as a LaunchConfiguration
    number_of_agents = LaunchConfiguration('number_of_agents')

    def create_nodes(context):
        nodes = []
        number_of_agents_value = int(LaunchConfiguration('number_of_agents').perform(context))

        for i in range(1, number_of_agents_value + 1):
            agent_id = f'agent{i:03d}'
            namespace = agent_id

            node = GroupAction([
                GroupAction([
                    Node(
                        package='pbtm_ros2',
                        executable='pbtm_ros2_node',
                        name='pbtm_node',
                        namespace=namespace,
                        output='screen',
                        parameters=[
                            {'agent_id': agent_id},
                            {'send_command_rate': 20.0},
                            {'timeout': 2.0},
							{'takeoff_height': 5.0},
                            {'global_start_position': [1.0, 0.0, 0.0]},  # In NWU coordinate system
                            {'height_range': [1.0, 100.0]},
                            {'yaw_offset_rad': 0.0},
                            {'order': 4.0},
                            {'max_velocity': 4.5},
                            {'knot_division': 3}
                        ]
                    ),
                ]),
            ])
            nodes.append(node)
        return nodes

    return LaunchDescription([
        number_of_agents_arg,
        OpaqueFunction(function=create_nodes)
    ])
