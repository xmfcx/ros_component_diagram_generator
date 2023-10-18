import pathlib

import launch
import launch_ros
from rclpy.logging import get_logger

logger = get_logger("launch2json")

from typing import Text

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
        d["node_package"] = entity.final_attributes.package
        d["node_executable"] = entity.final_attributes.node_executable
        d["expanded_node_namespace"] = entity.final_attributes.node_namespace
        d["node_name"] = entity.node_name.split("/")[-1]

    if type(entity) is launch_ros.actions.LoadComposableNodes:
        assert isinstance(entity, launch_ros.actions.LoadComposableNodes)
        d["target_container"] = entity._LoadComposableNodes__final_target_container_name

    if type(entity) is launch_ros.descriptions.ComposableNode:
        assert isinstance(entity, launch_ros.descriptions.ComposableNode)
        d["package"] = entity.final_attributes.package
        d["node_plugin"] = entity.final_attributes.node_plugin
        d["node_namespace"] = entity.final_attributes.node_namespace
        d["node_name"] = entity.final_attributes.node_name
        d["target_container"] = entity.final_attributes.target_container

        if entity.final_attributes.params_files is not None:
            d["params_files"] = entity.final_attributes.params_files

        if entity.final_attributes.params_dicts is not None:
            d["params_dicts"] = entity.final_attributes.params_dicts

        if entity.final_attributes.params_descs is not None:
            d["params_descs"] = entity.final_attributes.params_descs

        if entity.final_attributes.remap_rules is not None:
            d["remap_rules"] = []
            for i, remap in enumerate(entity.final_attributes.remap_rules):
                d["remap_rules"].append(remap)
        if entity.final_attributes.remap_rules_global is not None:
            d["remap_rules_global"] = []
            for i, remap in enumerate(entity.final_attributes.remap_rules_global):
                d["remap_rules_global"].append(remap)


    if type(entity) is launch_ros.actions.Node:
        assert isinstance(entity, launch_ros.actions.Node)
        d["package"] = entity.final_attributes.package
        d["node_namespace"] = entity.final_attributes.node_namespace
        d["node_name"] = entity.final_attributes.node_name
        d["node_executable"] = entity.final_attributes.node_executable

        if entity.final_attributes.arguments is not None:
            d["arguments"] = entity.final_attributes.arguments

        if entity.final_attributes.params_files is not None:
            d["params_files"] = entity.final_attributes.params_files

        if entity.final_attributes.params_dicts is not None:
            for param_dict in entity.final_attributes.params_dicts:
                for key, value in param_dict.items():
                    if type(value) is not str:
                        continue
                    if "xml version=" in value:
                        xml_content = value
                        xml_file_path = 'output/xml_file.xml'  # Adjust the path and filename as necessary

                        # 1. Write the XML into a file
                        with open(xml_file_path, 'w') as xml_file:
                            xml_file.write(xml_content)
                        import os
                        # 2. Put the location of the file into the `param_dict`
                        param_dict[key] = os.path.abspath(xml_file_path)

            d["params_dicts"] = entity.final_attributes.params_dicts

        if entity.final_attributes.params_descs is not None:
            d["params_descs"] = entity.final_attributes.params_descs

        if entity.final_attributes.params_global_tuples is not None:
            d["params_global_tuples"] = entity.final_attributes.params_global_tuples

        if entity.final_attributes.params_global_files is not None:
            d["params_global_files"] = entity.final_attributes.params_global_files

        if entity.final_attributes.remap_rules is not None:
            d["remap_rules"] = []
            for i, remap in enumerate(entity.final_attributes.remap_rules):
                d["remap_rules"].append(remap)
        if entity.final_attributes.remap_rules_global is not None:
            d["remap_rules_global"] = []
            for i, remap in enumerate(entity.final_attributes.remap_rules_global):
                d["remap_rules_global"].append(remap)

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
