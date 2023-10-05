from ament_index_python.packages import get_package_share_path
from jinja2 import Template
from rclpy.logging import get_logger

logger = get_logger("launch2json")


def _get_all_entities(tree: dict):
    if tree.get("children"):
        entities = [tree["entity"]]
        for subtree in tree["children"]:
            entities.extend(_get_all_entities(subtree))
        return entities
    else:
        return [tree]


def get_component_kind(entity: dict):
    if entity["type"] == "IncludeLaunchDescription":
        return "folder"
    if entity["type"] == "GroupAction":
        return "folder"
    if entity["type"] == "ComposableNodeContainer":
        return "folder"
    if entity["type"] == "LoadComposableNodes":
        return "folder"
    if entity["type"] == "ComposableNode":
        return "node"
    if entity["type"] == "Node":
        return "node"
    if entity["type"] == "ExecuteProcess":
        return "node"
    return "card"


def get_component_id(entity: dict):
    if entity["type"] == "IncludeLaunchDescription":
        return f'{entity["type"]}_{entity["id"]}'
    if entity["type"] == "GroupAction":
        return f'{entity["type"]}_{entity["id"]}'
    if entity["type"] == "ComposableNodeContainer":
        return f'{entity["type"]}_{entity["id"]}'
    if entity["type"] == "LoadComposableNodes":
        return f'{entity["type"]}_{entity["id"]}'
    if entity["type"] == "ComposableNode":
        return f'{entity["type"]}_{entity["id"]}'
    if entity["type"] == "Node":
        return f'{entity["type"]}_{entity["id"]}'
    if entity["type"] == "ExecuteProcess":
        return f'{entity["type"]}_{entity["id"]}'
    return f'Unknown_{entity["type"]}_{entity["id"]}'


def get_component_style(entity: dict):
    if entity["type"] == "IncludeLaunchDescription":
        return "#Salmon"
    if entity["type"] == "GroupAction":
        return "#Pink"
    if entity["type"] == "ComposableNodeContainer":
        return "#LemonChiffon"
    if entity["type"] == "LoadComposableNodes":
        return "#LightGreen"
    if entity["type"] == "ComposableNode":
        return "#PaleTurquoise"
    if entity["type"] == "Node":
        return "#LightSkyBlue"
    if entity["type"] == "ExecuteProcess":
        return "#Wheat"
    if entity["type"] == "SetParameter":
        return "#Orange"
    if entity["type"] == "SetRemap":
        return "#Khaki"
    return "#Pink"


def create_entity_index_map(tree: dict):
    index_map = {}

    def update_index(tree):
        if tree.get("children"):
            index_map[str(tree["entity"])] = tree
            for subtree in tree["children"]:
                update_index(subtree)
        else:
            index_map[str(tree)] = tree

    update_index(tree)

    return index_map


def get_children(index_map: dict, entity: dict):
    index = index_map[str(entity)]
    if not index.get("children"):
        return []

    children = []
    for child in index["children"]:
        if child.get("children"):
            children.append(child["entity"])
        else:
            children.append(child)

    return children


def generate_launch_file(serializable_tree: dict):
    share = get_package_share_path("ros_component_diagram_generator")
    template_text = (share / "templates" / "launch_generated.jinja2").read_text()
    template = Template(template_text)

    entities = _get_all_entities(serializable_tree)

    # sort entities by their types
    entities = sorted(entities, key=lambda e: get_component_kind(e))

    for i, e in enumerate(entities):
        e["id"] = i

    index_map = create_entity_index_map(serializable_tree)

    # Target items are:
    # - ComposableNode
    # - Node
    # - ComposableNodeContainer
    # - SetParameter

    return template.render(
        {
            "entities": entities
        }
    )
