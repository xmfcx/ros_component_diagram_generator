import launch
import launch_ros
from rclpy.logging import get_logger

logger = get_logger("launch2json")


def _to_string(context, substitutions):
    from launch.utilities import normalize_to_list_of_substitutions
    from launch.utilities import perform_substitutions

    if substitutions is None:
        substitutions = ""

    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def _parse_entity_tree(entity: launch.LaunchDescriptionEntity, context: launch.LaunchContext):
    # print("parsing entity: " + entity.__class__.__name__)
    if isinstance(entity, launch.Action):
        if entity.condition is not None:
            if not entity.condition.evaluate(context):
                return None



    sub_entities = None
    try:
        sub_entities = entity.visit(context)
        # if sub_entities is not None:
        #     for sub_entity in sub_entities:
        #         # if sub_entity.condition is not None:
        #         #     if not sub_entity.condition.evaluate(context):
        #         print("  - child: " + sub_entity.__class__.__name__)
    except Exception as e:
        logger.warn(f"error in {entity.__class__}: {e}")

    if isinstance(entity, launch_ros.actions.Node):
        entity._Node__package = _to_string(context, entity._Node__package)
        entity._Node__node_executable = _to_string(context, entity._Node__node_executable)
        entity._Node__node_name = _to_string(context, entity.node_name)

        # print("node_name: " + entity._Node__node_name)
        #
        # if entity.expanded_remapping_rules is not None:
        #     for i, remap in enumerate(entity.expanded_remapping_rules):
        #         # from launch_ros import remap_rule_type
        #         # remap_rule_type.RemapRule
        #         item_1 = _to_string(context, remap[0])
        #         item_2 = _to_string(context, remap[1])
        #         print(f"remap: {item_1}:={item_2}")

    if isinstance(entity, launch_ros.actions.SetParameter):
        assert isinstance(entity, launch_ros.actions.SetParameter)
        entity._MFC__param_name = _to_string(context, entity.name)

    if isinstance(entity, launch_ros.actions.LoadComposableNodes):
        nodes = []
        for n in entity._LoadComposableNodes__composable_node_descriptions:
            assert isinstance(n, launch_ros.descriptions.ComposableNode)
            nodes.append(n)
        return {
            "entity": entity,
            "children": nodes,
        }

    if sub_entities is None:
        return entity

    if isinstance(entity, launch.actions.IncludeLaunchDescription):
        logger.debug(entity._get_launch_file())

    children = [_parse_entity_tree(sub_entity, context) for sub_entity in sub_entities]

    return {
        "entity": entity,
        "children": children,
    }


def create_entity_tree(
        root_entity: launch.LaunchDescriptionEntity,
        launch_service: launch.LaunchService,
):
    import asyncio
    import logging

    loop = asyncio.get_event_loop()
    launch_service.context._set_asyncio_loop(loop)
    print("parsing the entity tree")
    entity_tree = _parse_entity_tree(root_entity, launch_service.context)
    print("parsed the entity tree")

    tasks = asyncio.all_tasks(loop)
    for task in tasks:
        task.cancel()

    logging.root.setLevel("CRITICAL")
    loop.create_task(launch_service.run_async())
    loop.run_until_complete(asyncio.sleep(0))
    shutdown_task = loop.create_task(launch_service.shutdown())
    loop.run_until_complete(shutdown_task)
    logging.root.setLevel("INFO")

    return entity_tree
