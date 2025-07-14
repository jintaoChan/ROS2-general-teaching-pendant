from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("auto_store_robot_moveit_config", package_name="auto_store_robot_moveit_config").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
