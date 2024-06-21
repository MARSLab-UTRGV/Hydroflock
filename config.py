import xml.etree.ElementTree as ET
import os

def read_config(file_path):

    file_path = "./experiments/"+file_path
    """Read and parse the ARGoS configuration file."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The configuration file '{file_path}' does not exist.")
    
    tree = ET.parse(file_path)
    root = tree.getroot()
    return tree, root

def save_config(tree, file_path):
    """Save the updated configuration to a file."""
    tree.write(file_path, encoding='utf-8', xml_declaration=True)
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
    
    # Define wall parameters
    walls = [
        {"id": "wall_north", "position": "0,4.5,0.25", "size": "5,0.1,0.5"},
        {"id": "wall_south", "position": "0,-4.5,0.25", "size": "5,0.1,0.5"},
        {"id": "wall_east", "position": "4.5,0,0.25", "size": "0.1,10,0.5"},
        {"id": "wall_west", "position": "-4.5,0,0.25", "size": "0.1,10,0.5"}
    ]
    
    # Add walls to the arena
    for wall in walls:
        wall_element = ET.SubElement(arena, "box", attrib={
            "id": wall["id"],
            "position": wall["position"],
            "size": wall["size"],
            "movable": "false"
        })

if __name__ == "__main__":

    config_file = 'hydroflock_dev.argos'
    try:
        tree, root = read_config(config_file)
    except FileNotFoundError as e:
        print(e)
        exit(1)

    update_gain(root, 1250)

    save_config(tree, config_file)
