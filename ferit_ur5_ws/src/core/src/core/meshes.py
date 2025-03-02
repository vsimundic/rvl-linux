import numpy as np
import pyassimp
import os
from typing import Union
import xml.etree.ElementTree as ET

def convert_to_dae(input_file: str, output_file: str = None, object_name: str = None, print_msgs=False) -> Union[str, None]:
    """
    Converts a .ply, .obj, or .stl file to .dae format.
    
    Args:
        input_file (str): Path to the input .ply, .obj, or .stl file.
        output_file (str, optional): Path to the output .dae file. Defaults to None.
        object_name (str, optional): Name to assign to the object in the scene.

    Returns:
        str: Path to the .dae file if conversion is successful, else None.
    """

    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file '{input_file}' not found.")

    if not input_file.lower().endswith(('.ply', '.obj', '.stl')):
        raise ValueError("Input file must be a .ply, .obj, or .stl file.")

    if output_file is None:
        output_file = os.path.splitext(input_file)[0] + ".dae"

    try:
        # Load the input file using PyAssimp
        scene = pyassimp.load(input_file)

        # Set object name if provided
        if object_name:
            if scene.rootnode:
                scene.rootnode.name = object_name
            else:
                raise ValueError("Scene root node is missing. Cannot set object name.")
            
            # Set the name for each mesh in the scene
            for mesh in scene.meshes:
                mesh.name = object_name

        # Fix rotation (Apply a transform to align Z-up STL to Y-up DAE)
        rotation_matrix = [
            [1, 0, 0, 0],  # Keep X the same
            [0, 1, 0, 0], # Rotate Y to Z
            [0, 0, 1, 0],  # Rotate Z to Y
            [0, 0, 0, 1],  # Homogeneous coordinates
        ]
        scene.rootnode.transformation = rotation_matrix

        # Export to .dae
        pyassimp.export(scene, output_file, 'collada')
        pyassimp.release(scene)
        if print_msgs:
            print(f"Converted '{input_file}' to '{output_file}' successfully.")
        return output_file

    except Exception as e:
        if print_msgs:
            print(f"Conversion failed: {e}")
        return None
    
def fix_dae_up_axis(dae_file: str, axis_up:str = 'Z_UP', output_file: str = None, print_msgs=False):
    """
    Modifies the COLLADA (.dae) file to set up_axis to Z_UP.

    Args:
        dae_file (str): Path to the input .dae file.
        axis_up (str): Select which axis to set as up. Default is 'Z_UP'. Other options are 'X_UP' and 'Y_UP'.
        output_file (str, optional): Path to save the modified .dae file.
    
    Returns:
        str: Path to the modified .dae file.
    """
    if axis_up not in {'X_UP', 'Y_UP', 'Z_UP'}:
        raise ValueError("Invalid axis_up value. Choose from 'X_UP', 'Y_UP', or 'Z_UP'.")

    if output_file is None:
        output_file = dae_file  # Overwrite the original file

    # Load the DAE XML file
    tree = ET.parse(dae_file)
    root = tree.getroot()

    # Define COLLADA namespace (commonly used in DAE files)
    namespace = "http://www.collada.org/2005/11/COLLADASchema"
    ET.register_namespace('', namespace)  # Preserve the namespace in the output

    # Find the up_axis tag
    up_axis_tag = root.find(f".//{{{namespace}}}up_axis")
    
    if up_axis_tag is not None:
        # Change the up_axis value
        up_axis_tag.text = axis_up
        tree.write(output_file, encoding="utf-8", xml_declaration=True)
        if print_msgs:
            print(f"Updated up_axis to {axis_up} in {output_file}")
    else:
        if print_msgs:
            print("Could not find <up_axis> tag in the DAE file.")

    return output_file