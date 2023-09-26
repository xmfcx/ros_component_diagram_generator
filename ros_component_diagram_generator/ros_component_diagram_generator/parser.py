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
    if isinstance(entity, launch.Action):
        if entity.condition is not None:
            if not entity.condition.evaluate(context):
                return None

    sub_entities = None
    try:
        sub_entities = entity.visit(context)
    except Exception as e:
        logger.warn(f"error in {entity.__class__}: {e}")

    if isinstance(entity, launch_ros.actions.Node):
        entity._Node__package = _to_string(context, entity._Node__package)
        entity._Node__node_executable = _to_string(context, entity._Node__node_executable)

    if isinstance(entity, launch_ros.actions.SetParameter):
        assert isinstance(entity, launch_ros.actions.SetParameter)
        entity._MFC__param_name = _to_string(context, entity.name)

    if isinstance(entity, launch_ros.actions.LoadComposableNodes):
        nodes = []
        for n in entity._LoadComposableNodes__composable_node_descriptions:
            assert isinstance(n, launch_ros.descriptions.ComposableNode)
            n._ComposableNode__package = _to_string(context, n.package)
            n._ComposableNode__node_plugin = _to_string(context, n.node_plugin)
            n._ComposableNode__node_namespace = _to_string(context, n.node_namespace)
            n._ComposableNode__node_name = _to_string(context, n.node_name)
            # n._ComposableNode__params = _to_string(context, n.parameters)
            # n._ComposableNode__remaps = _to_string(context, n.remappings)

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
