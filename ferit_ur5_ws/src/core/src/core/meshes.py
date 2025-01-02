import numpy as np
import pyassimp
import os
from typing import Union

def convert_to_dae(input_file: str, output_file: str=None, object_name:str=None) -> Union[str, None]:
    """
    Converts a .ply or .obj file to .dae format.
    
    Args:
        input_file (str): Path to the input .ply or .obj file.
        output_file (str): Path to the output .dae file (optional).
    
    Returns:
        str: Path to the .dae file if conversion is successful.
    """

    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file '{input_file}' not found.")

    if not input_file.lower().endswith(('.ply', '.obj')):
        raise ValueError("Input file must be a .ply or .obj file.")

    if output_file is None:
        output_file = os.path.splitext(input_file)[0] + ".dae"

    # Attempt conversion with PyAssimp
    if pyassimp:
        try:
            # Load the input file
            scene = pyassimp.load(input_file)

           # Set the object name if provided
            if object_name:
                if scene.rootnode:
                    scene.rootnode.name = object_name
                else:
                    raise ValueError("Scene root node is missing. Cannot set object name.")
                
                # Set the name for each mesh in the scene
                for mesh in scene.meshes:
                    mesh.name = object_name
            
            # Export to .dae
            pyassimp.export(scene, output_file, 'collada')
            pyassimp.release(scene)
            print(f"Converted '{input_file}' to '{output_file}' using PyAssimp.")
            return output_file
        except Exception as e:
            print(f"PyAssimp failed: {e}")
            return None
