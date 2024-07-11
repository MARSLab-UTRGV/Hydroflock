import xml.etree.ElementTree as ET
from xml.dom import minidom
from colorama import init, Fore, Style
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

def align_controller_tps(root):
    """ Computes the timeStep attribute for the controller settings using the experiment ticks_per_second attribute """

    for experiment in root.findall(".//framework/experiment"):
        experiment_tps = float(experiment.get('ticks_per_second'))

    timeStep = 1.0/experiment_tps

    for controller in root.findall(".//controllers/footbot_hydroflock_controller/params/flocking"):
        controller.set('time_step', timeStep)

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
    wall_height = 1.0
    
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

def add_cylinder_obstacles(root, test_case):
    """
    Adds cylinder obstacles to the ARGoS configuration based on the specified test case.
    
    Parameters:
    - root: The root element of the XML tree.
    - test_case: An integer specifying the test case configuration.
    """
    arena = root.find('.//arena')
    if arena is None:
        raise ValueError("No arena element found in the configuration file")

    if test_case == 0:
        distribute = ET.SubElement(arena, 'distribute')
        position = ET.SubElement(distribute, 'position', attrib={'method': 'constant', 'values': '0,0,0'})
        orientation = ET.SubElement(distribute, 'orientation', attrib={'method': 'constant', 'values': '0,0,0'})
        entity = ET.SubElement(distribute, 'entity', attrib={'quantity': '1', 'max_trials': '100'})
        ET.SubElement(entity, 'cylinder', attrib={'id': 'cylinder_center', 'radius': '0.25', 'height': '0.5', 'movable': 'false'})
    elif test_case == 1:
        x_positions = [-1.5, -0.5, 0.5, 1.5]
        for i, x in enumerate(x_positions):
            distribute = ET.SubElement(arena, 'distribute')
            position = ET.SubElement(distribute, 'position', attrib={'method': 'constant', 'values': f'{x},0,0'})
            orientation = ET.SubElement(distribute, 'orientation', attrib={'method': 'constant', 'values': '0,0,0'})
            entity = ET.SubElement(distribute, 'entity', attrib={'quantity': '1', 'max_trials': '1'})
            ET.SubElement(entity, 'cylinder', attrib={'id': f'cylinder_{i}', 'radius': '0.25', 'height': '0.5', 'movable': 'false'})
    else:
        raise ValueError(f"Unknown test case: {test_case}")

def test_communication(root, set, test_case=0):
    """
    Turn communication testing on/off. This funciton is required to be called even if 'set' is False.

    Parameters:
    - root: The root element of the XML tree.
    - set: Boolean to specify on/off.
    - test_case: An integer specifying the test case configuration (default parameter for cases when 'set' is False).

    We will use this to test communication protocols without the bots moving. We can set specific topologies to test.
    """

    def bool_to_string(value):
        """
        Helper function to convert boolean to string "true" or "false"
        """
        return "true" if value else "false"

    for controller in root.findall(".//controllers/footbot_hydroflock_controller/params/settings"):
        controller.set('communication_test', bool_to_string(set))

    if set:
        if test_case == 0: 
            footbot_vertical_distribution(root)

def footbot_vertical_distribution(root, num_footbots=6, spacing=0.5):
    """
    Distribute foot-bots in a vertical line with specified spacing, centered in the arena.
    
    Parameters:
    - root: The root element of the XML tree
    - num_footbots: The number of foot-bots to distribute
    - spacing: The spacing between foot-bots (default is 0.5)
    """
    # Calculate the total height needed for the vertical line
    total_height = (num_footbots - 1) * spacing
    # Calculate the center y position to center the line of foot-bots
    center_y = 0  # Assuming the arena center is at y=0

    # Create the distribute element
    distribute = ET.SubElement(root.find('.//arena'), 'distribute')

    # Set the grid distribution parameters
    position = ET.SubElement(distribute, 'position', attrib={
        'center': f'0,{center_y},0',
        'distances': f'0,{spacing},0',
        'layout': f'1,{num_footbots},1',
        'method': 'grid'
    })
    orientation = ET.SubElement(distribute, 'orientation', attrib={'method': 'constant', 'values': '0,0,0'})
    entity = ET.SubElement(distribute, 'entity', attrib={'quantity': str(num_footbots), 'max_trials': '100'})
    
    footbot = ET.SubElement(entity, 'foot-bot', attrib={'id': 'fb',
                                                        'omnidirectional_camera_aperture': '80',
                                                        'rab_range': '3',
                                                        'rab_data_size': '64',})
    controller = ET.SubElement(footbot, 'controller', attrib={'config': 'ffc'})

    return root

def footbot_default_distribution(root):
    """
    Adds the distribution for foot-bots to the ARGoS configuration. **Currently just default what was in the argos example**
    
    Parameters:
    - root: The root element of the XML tree.
    """
    arena = root.find('.//arena')
    if arena is None:
        raise ValueError("No arena element found in the configuration file")

    distribute = ET.SubElement(arena, 'distribute')
    position = ET.SubElement(distribute, 'position', attrib={'method': 'uniform', 'min': '-0.5,-4,0', 'max': '0.5,-3,0'})
    orientation = ET.SubElement(distribute, 'orientation', attrib={'method': 'gaussian', 'mean': '0,0,0', 'std_dev': '360,0,0'})
    
    entity = ET.SubElement(distribute, 'entity', attrib={'quantity': '10', 'max_trials': '100'})
    footbot = ET.SubElement(entity, 'foot-bot', attrib={'id': 'fb', 'omnidirectional_camera_aperture': '80'})
    ET.SubElement(footbot, 'controller', attrib={'config': 'ffc'})

def clear_entity_distributions(root):
    """
    Clears all entities distributed using the "distribute" element (including footbots)
    
    Parameters:
    - root: The root element of the XML tree.
    """
    arena = root.find('.//arena')
    if arena is None:
        raise ValueError("No arena element found in the configuration file")

    # Remove all existing distribute elements
    for distribute in arena.findall('distribute'):
        arena.remove(distribute)

def check_config(root):
    init(autoreset=True)
    try:
        # Check for distribute elements containing foot-bot elements
        distribute_elements = root.findall(".//distribute")
        has_footbots = False
        for distribute in distribute_elements:
            if distribute.find(".//foot-bot") is not None:
                has_footbots = True
                break
        
        if not has_footbots:
            print(Fore.RED + "WARNING: No foot-bot distribution found in the configuration file.")
        
    except ET.ParseError as e:
        print(f"Error parsing the XML file: {e}")

######################################################################################################################

def initialize(fp, comms_test=False):

    try:
        tree, root = read_config(fp)
    except FileNotFoundError as e:
        print(e)
        exit(1)

    clear_entity_distributions(root) # This must go first before distributing anything including footbot entities

    add_enclosing_walls(root)
    
    test_communication(root, comms_test)

    return tree, root

def finalize(tree, root, fp):

    check_config(root)

    save_config(tree, root, fp)

######################################################################################################################

if __name__ == "__main__":

    config_fp = './experiments/hydroflock_dev.argos'
    
    tree, root = initialize(config_fp, comms_test=True)

    # footbot_default_distribution(root)

    # add_cylinder_obstacles(root, test_case = 1)

    finalize(tree, root, config_fp)
