import xml.etree.ElementTree as ET
from xml.dom import minidom
import os

def read_config(file_path):

    """Read and parse the ARGoS configuration file."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The configuration file '{file_path}' does not exist.")
    
    tree = ET.parse(file_path)
    root = tree.getroot()

    return tree, root

def save_config(tree, root, file_path):
    """Prettify and save the updated configuration to a file."""

    xml_str = ET.tostring(root, encoding='utf-8')
    pretty_xml = minidom.parseString(xml_str).toprettyxml(indent="    ")

    # Remove unnecessary blank lines
    pretty_xml = "\n".join([line for line in pretty_xml.split("\n") if line.strip()])

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(pretty_xml)

    # tree.write(file_path, encoding='utf-8', xml_declaration=True)
    print(f"Configuration saved to {file_path}")

def update_gain(root, new_gain):
    """Update the gain parameter in the flocking settings."""
    gain_updated = False
    for controller in root.findall(".//controllers/footbot_hydroflock_controller/params/flocking"):
        current_gain = controller.get('gain')
        if current_gain is not None and float(current_gain) != float(new_gain):
            controller.set('gain', str(new_gain))
            gain_updated = True

    if gain_updated:
        print(f"Updated gain to {new_gain}")

def add_enclosing_walls(root):
    """Add walls that enclose the arena."""
    arena = root.find('.//arena')

    # Extract the arena size
    arena_size = arena.get('size')
    width, length, height = map(float, arena_size.split(','))

    # Define wall parameters based on arena size
    wall_thickness = 0.1
    wall_height = height
    
    # Define wall parameters
    walls = [
        {"id": "wall_north", "position": f"0,{length / 2},0", "size": f"{width+wall_thickness},{wall_thickness},{wall_height}"},
        {"id": "wall_south", "position": f"0,{-length / 2},0", "size": f"{width+wall_thickness},{wall_thickness},{wall_height}"},
        {"id": "wall_east", "position": f"{width / 2},0,0", "size": f"{wall_thickness},{length},{wall_height}"},
        {"id": "wall_west", "position": f"{-width / 2},0,0", "size": f"{wall_thickness},{length},{wall_height}"}
    ]

    # Remove existing walls
    for wall in arena.findall("box"):
        arena.remove(wall)
    
    # Add walls to the arena
    for wall in walls:
        wall_element = ET.SubElement(arena, "box", attrib={
            "id": wall["id"],
            "size": wall["size"],
            "movable": "false"
        })
        body_element = ET.SubElement(wall_element, "body", attrib={
            "position": wall["position"],
            "orientation": "0,0,0"
        })

    

if __name__ == "__main__":

    config_file_path = './experiments/hydroflock_dev.argos'
    try:
        tree, root = read_config(config_file_path)
    except FileNotFoundError as e:
        print(e)
        exit(1)

    update_gain(root, 1250)

    add_enclosing_walls(root)

    save_config(tree, root, config_file_path)
