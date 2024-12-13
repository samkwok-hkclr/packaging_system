from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    NUMBER_OF_PACKAGING_MACHINE = 1

    for i in range(1, NUMBER_OF_PACKAGING_MACHINE + 1):
        node = Node(
                package='packaging_machine_control_system',
                namespace=f'packaging_machine_{i}',
                executable='packaging_machine_node',
                # name=f'packaging_machine_node_{i}',
                parameters=[
                    {'packaging_machine_id': i},
                    {'simulation': False},

                    {'vendor_id': 0x471},
                    {'product_id': 0x55},
                    {'serial': "0003B0000000"},
                    {'endpoint_in': 0x82},
                    {'endpoint_out': 0x02},
                    {'timeout': 1000},
                    {'dots_per_mm': 12},
                    {'direction': 0},
                    {'total': 0},
                    {'interval': 1000},
                    {'offset_x': False},
                    {'offset_y': False},
                ],
                respawn=True,
                respawn_delay=5,
                output='screen',
            )
        ld.add_action(node)

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