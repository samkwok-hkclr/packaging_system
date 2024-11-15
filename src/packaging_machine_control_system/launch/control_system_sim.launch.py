from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    NUMBER_OF_PACKAGING_MACHINE = 1

    for i in range(1, NUMBER_OF_PACKAGING_MACHINE + 1):
        ld.add_action(
            Node(
                package='packaging_machine_control_system',
                namespace=f'packaging_machine_{i}',
                executable='packaging_machine_node',
                # name=f'packaging_machine_node_{i}',
                parameters=[
                    {'packaging_machine_id': i},
                    {'simulation': True}
                ],
                respawn=True,
                respawn_delay=5,
                output='screen',
            )
        )

    manager = Node(
        package='packaging_machine_control_system',
        executable='packaging_machine_manager',
        name='packaging_machine_manager',
        parameters=[{'packaging_machines': NUMBER_OF_PACKAGING_MACHINE}],
        respawn=True,
        respawn_delay=5,
        output='screen',
    )
    ld.add_action(manager)

    return ld