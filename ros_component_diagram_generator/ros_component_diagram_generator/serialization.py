import pathlib

from numpy.testing._private.parameterized import param

import launch
import launch_ros
from launch_ros import parameters_type
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from rclpy.logging import get_logger

logger = get_logger("launch2json")

from typing import Text


def _to_string(context: launch.LaunchContext,
               substitutions: launch.some_substitutions_type.SomeSubstitutionsType) -> Text:
    from launch.utilities import normalize_to_list_of_substitutions
    from launch.utilities import perform_substitutions

    if substitutions is None:
        substitutions = ""

    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def _make_entity_serializable(entity: launch.LaunchDescriptionEntity, context: launch.LaunchContext):
    import re

    d = {}
    d["type"] = entity.__class__.__name__
    if type(entity) is launch.actions.IncludeLaunchDescription:
        assert isinstance(entity, launch.actions.IncludeLaunchDescription)
        package_name_pattern = r".*/(\w+)/share/.*"
        d["package"] = re.match(package_name_pattern, entity._get_launch_file_directory())[1]
        d["file_name"] = entity._get_launch_file().split("/")[-1]
        d["launch_description_source.location"] = entity.launch_description_source.location

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
        d["node_plugin"] = entity.node_plugin
        d["node_namespace"] = entity.node_namespace
        d["node_name"] = entity.node_name

        import yaml

        from launch_ros.utilities import evaluate_parameters

        if entity.parameters is not None:
            evaluated_parameters = evaluate_parameters(context, entity.parameters)
            for params in evaluated_parameters:
                if isinstance(params, dict):
                    yaml_param = yaml.dump(params, default_flow_style=False)
                    print(yaml_param)
                elif isinstance(params, pathlib.Path):
                    print("path: " + str(params))
                elif isinstance(params, launch_ros.descriptions.Parameter):
                    name, value = params.evaluate(context)
                    print("name: " + str(name) + " value: " + str(value))
                else:
                    raise RuntimeError('invalid normalized parameters {}'.format(repr(params)))






        #
        # if entity.parameters is not None:
        #     for i, param in enumerate(entity.parameters):
        #         if isinstance(param, parameters_type.ParameterFile):
        #             if(type(param.param_file) is pathlib.PosixPath):
        #                 d[f"{i} - Param File: "] = str(param.param_file)
        #             else:
        #                 print("Couldn't handle the type: " + str(type(param.param_file)))
        # #         # else:
        # #         #     print("other type: " + str(type(param)))
        #         elif isinstance(param, dict):
        #             print(yaml.dump(evaluate_parameters_dict(context, param)))
        #
        # #             for key, value in param.items():
        # #                 print("key: " + _to_string(context, key))
        # #                 print("type_v: " + str(type(value)))
        # #                 assert isinstance(value, parameters_type.ParameterValue)
        # #                 # d[f"{i} - Param Dict: {key}"] = _to_string(context, value)


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

        # print(str_command)
        # d["cmd"] = "<back:white><font:Courier New>" + str_command + "</font></back>"
        # # d["cwd"] = entity.cwd

    if type(entity) is launch.actions.ExecuteProcess:
        assert isinstance(entity, launch.actions.ExecuteProcess)
        d["name"] = entity.name

    if type(entity) is launch_ros.actions.SetParameter:
        assert isinstance(entity, launch_ros.actions.SetParameter)
        d["param_name"] = entity._MFC__param_name

    return d


def make_entity_tree_serializable(tree: dict | object, context: launch.LaunchContext):
    if not isinstance(tree, dict):
        return _make_entity_serializable(tree, context=context)

    d = {}
    d["entity"] = _make_entity_serializable(tree["entity"], context=context)
    if isinstance(tree["children"], list):
        d["children"] = [
            make_entity_tree_serializable(child, context) for child in tree["children"]
        ]

    return d
