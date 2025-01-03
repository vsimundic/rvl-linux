#!/usr/bin/env python3

import os
import subprocess
from pathlib import Path
from tqdm import tqdm

def convert_urdf_to_dae(urdf_file, output_dir):
    """Converts a URDF file to a DAE file using collada_urdf."""
    output_file = Path(output_dir) / (Path(urdf_file).stem + ".dae")
    try:
        subprocess.check_call(["rosrun", "collada_urdf", "urdf_to_collada", urdf_file, str(output_file)])
        # print(f"Converted {urdf_file} to {output_file}")
    except subprocess.CalledProcessError as e:
        # print(f"Failed to convert {urdf_file} to DAE: {e}")
        pass
    return output_file

def convert_stl_to_dae(stl_file, output_dir):
    """Converts an STL file to a DAE file using meshlabserver."""
    output_file = Path(output_dir) / (Path(stl_file).stem + ".dae")
    try:
        subprocess.check_call([
            "meshlabserver",
            "-i", stl_file,
            "-o", str(output_file),
            "-om", "vn", "fc"
        ])
        # print(f"Converted {stl_file} to {output_file}")
    except subprocess.CalledProcessError as e:
        # print(f"Failed to convert {stl_file} to DAE: {e}")
        pass
    return output_file

def main():
    input_dir = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/tsr_data/test_configs'
    output_dir = '/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/tsr_data/test_configs'

    Path(output_dir).mkdir(parents=True, exist_ok=True)

    for file in tqdm(os.listdir(input_dir)):
        file_path = Path(input_dir) / file
        if file.endswith(".urdf"):
            convert_urdf_to_dae(str(file_path), output_dir)
        # if file.endswith(".stl"):
        #     convert_stl_to_dae(str(file_path), output_dir)
        else:
            # print(f"Skipping unsupported file: {file}")
            pass

if __name__ == "__main__":
    main()