import launch
import launch_ros
from rclpy.logging import get_logger

logger = get_logger("launch2json")


def _make_entity_serializable(entity: launch.LaunchDescriptionEntity):
    import re

    d = {}
    d["type"] = entity.__class__.__name__
    if type(entity) is launch.actions.IncludeLaunchDescription:
        package_name_pattern = r".*/(\w+)/share/.*"
        d["package"] = re.match(package_name_pattern, entity._get_launch_file_directory())[1]
        d["file_name"] = entity._get_launch_file().split("/")[-1]

    if type(entity) is launch.actions.GroupAction:
        d["scoped"] = entity._GroupAction__scoped
        d["forwarding"] = entity._GroupAction__forwarding

    if type(entity) is launch_ros.actions.ComposableNodeContainer:
        d["package"] = entity.node_package
        d["executable"] = entity.node_executable
        d["namespace"] = entity._Node__expanded_node_namespace.replace(
            launch_ros.actions.Node.UNSPECIFIED_NODE_NAMESPACE, "/"
        )
        d["name"] = entity._Node__expanded_node_name

    if type(entity) is launch_ros.actions.LoadComposableNodes:
        d["target_container"] = entity._LoadComposableNodes__final_target_container_name

    if type(entity) is launch_ros.descriptions.ComposableNode:
        d["package"] = entity.package
        d["plugin"] = entity.node_plugin
        d["namespace"] = entity.node_namespace
        d["name"] = entity.node_name

    if type(entity) is launch_ros.actions.Node:
        assert isinstance(entity, launch_ros.actions.Node)
        d["node_package"] = entity.node_package
        d["node_executable"] = entity.node_executable
        d["expanded_node_namespace"] = entity.expanded_node_namespace
        d["name"] = entity.name
        d["node_name"] = entity.node_name

        try:
            str_remapping_rules = str(set(entity.expanded_remapping_rules))
        except:
            str_remapping_rules = "None"

        d["expanded_remapping_rules"] = str_remapping_rules
        # print(entity.env)
        # d["env"] = entity.env
        # d["additional_env"] = entity.additional_env

        str_command = str(entity.cmd)
        # replace [] with {}
        str_command = str_command.replace("[", "{")
        str_command = str_command.replace("]", "}")

        print(str_command)
        d["cmd"] = "<back:white><font:Courier New>" + str_command + "</font></back>"
        # d["cwd"] = entity.cwd

    if type(entity) is launch.actions.ExecuteProcess:
        d["name"] = entity.name

    return d


def make_entity_tree_serializable(tree: dict | object, context: launch.LaunchContext):
    if not isinstance(tree, dict):
        return _make_entity_serializable(tree)

    d = {}
    d["entity"] = _make_entity_serializable(tree["entity"])
    if isinstance(tree["children"], list):
        d["children"] = [
            make_entity_tree_serializable(child, context) for child in tree["children"]
        ]

    return d
