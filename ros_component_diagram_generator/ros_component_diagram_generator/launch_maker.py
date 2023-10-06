from ament_index_python.packages import get_package_share_path
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

    entities = _get_all_entities(serializable_tree)

    # sort entities by their types
    entities = sorted(entities, key=lambda e: get_component_kind(e))

    def format_param(value):
        # If value is a float, format without scientific notation
        if isinstance(value, float):
            formatted_value = '{:f}'.format(value)
            # If there's a decimal point and trailing zeros
            if '.' in formatted_value:
                # Remove all but one trailing zero and then remove trailing dots, if any
                formatted_value = formatted_value.rstrip('0')
                if formatted_value[-1] != '0':
                    formatted_value += '0'
                formatted_value = formatted_value.rstrip('.')
            value = formatted_value

        # For other types (assuming they can be converted to strings with str())
        value = str(value)

        # Replace parentheses with brackets
        value = value.replace("(", "[").replace(")", "]")

        return value

    from jinja2 import Environment
    env = Environment(
        loader=None  # No loader since we're providing the template string directly
    )
    env.filters['format_param'] = format_param

    template = env.from_string(template_text)

    return template.render(
        {
            "entities": entities
        }
    )
