import launch
import launch_ros
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from rclpy.logging import get_logger

logger = get_logger("launch2json")


def _make_entity_serializable(entity: launch.LaunchDescriptionEntity):
    import re

    d = {}
    d["type"] = entity.__class__.__name__
    if type(entity) is launch.actions.IncludeLaunchDescription:
        assert isinstance(entity, launch.actions.IncludeLaunchDescription)
        package_name_pattern = r".*/(\w+)/share/.*"
        d["package"] = re.match(package_name_pattern, entity._get_launch_file_directory())[1]
        d["file_name"] = entity._get_launch_file().split("/")[-1]
        d["launch_description_source.location"] = entity.launch_description_source.location

        # iterate over launch_arguments
        d["launch_arguments count"] = len(entity.launch_arguments)

        print("package: " + d["package"])
        print("entity.launch_description_source.location: " + entity.launch_description_source.location)
        print("launch_arguments count: " + str(len(entity.launch_arguments)))

        for arg in entity.launch_arguments:
            if type(arg[0]) is str:
                if type(arg[1]) is str:
                    print(arg[0] + ": " + arg[1])
                # if type(arg[1]) is LaunchConfiguration:
                #     x = arg[1]
                #     assert isinstance(x, LaunchConfiguration)
                #     for y in x.perform():
                #         print(arg[0] + ": " + y)
                else:
                    print("!!!!!!!!!!!!" + type(arg[1]).__name__)
            elif type(arg[0]) is list:
                for whatisthis in arg[0]:
                    if isinstance(whatisthis, TextSubstitution):
                        print("-- " + whatisthis.text + ": " + whatisthis.describe())
                    else:
                        print("!!!!!!!!!!!!" + type(whatisthis).__name__)
            else:
                print("hmm...: " + type(arg[0]).__name__)

        try:
            str_args = str(entity.launch_arguments)
        except:
            str_args = "None"

        # replace [] with {}
        str_args = str_args.replace("[", "{")
        str_args = str_args.replace("]", "}")

        d["launch_arguments"] = str_args

    if type(entity) is launch.actions.GroupAction:
        assert isinstance(entity, launch.actions.GroupAction)
        d["scoped"] = entity._GroupAction__scoped
        d["forwarding"] = entity._GroupAction__forwarding

    if type(entity) is launch_ros.actions.ComposableNodeContainer:
        assert isinstance(entity, launch_ros.actions.ComposableNodeContainer)
        d["node_package"] = entity.node_package
        d["node_executable"] = entity.node_executable
        d["expanded_node_namespace"] = entity.expanded_node_namespace
        d["node_name"] = entity.node_name
        d["name"] = entity.name

    if type(entity) is launch_ros.actions.LoadComposableNodes:
        assert isinstance(entity, launch_ros.actions.LoadComposableNodes)
        d["target_container"] = entity._LoadComposableNodes__final_target_container_name

    if type(entity) is launch_ros.descriptions.ComposableNode:
        assert isinstance(entity, launch_ros.descriptions.ComposableNode)
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
        #
        # try:
        #     str_remapping_rules = str(set(entity.expanded_remapping_rules))
        # except:
        #     str_remapping_rules = "None"
        #
        # d["expanded_remapping_rules"] = str_remapping_rules
        # # print(entity.env)
        # # d["env"] = entity.env
        # # d["additional_env"] = entity.additional_env
        #
        # str_command = str(entity.cmd)
        # # replace [] with {}
        # str_command = str_command.replace("[", "{")
        # str_command = str_command.replace("]", "}")
        #
        # print(str_command)
        # d["cmd"] = "<back:white><font:Courier New>" + str_command + "</font></back>"
        # # d["cwd"] = entity.cwd

    if type(entity) is launch.actions.ExecuteProcess:
        assert isinstance(entity, launch.actions.ExecuteProcess)
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
