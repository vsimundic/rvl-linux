# Door opening pipeline from Human demonstration

This repository contains a Docker container which involves:
- Robotic Vision Library (RVL),
- ROS with packages for controlling the UR5 robot and
- TensorMask from detectron2.

## Installation

### Docker installation

You will need a PC that supports **Nvidia drivers and CUDA** (don't need to install them on your own) in order to launch TensorMask.

Ubuntu: 
Install Docker from the [official page](https://docs.docker.com/engine/install/ubuntu/). Don't forget to do the [postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/).

Windows:
Install Docker from the [official page](https://docs.docker.com/desktop/install/windows-install/). To run the necessary shell scripts, you can use [WSL](https://learn.microsoft.com/en-us/windows/wsl/install).


### Container setup (Linux)

Clone and enter the repository:
```bash
git clone https://gitlab.com/vsimundic/rvl-ur5-detectron2-docker

cd rvl-ur5-detectron2-docker
```

Run the shell script to build the Docker image:
```bash
./build_docker.sh
```

To be able to show any windows from the container, run:
```bash
xhost +
```
This command should be ran on every login. To avoid it, you can add it to .bashrc:
```bash
echo 'xhost +' >> ~/.bashrc 
```

After installation, run the Docker container:
```bash
./run_docker.sh
```
The scripts will first stop and remove the current container if it exists. If it doesn't it will output an error that it can't stop or remove the container, but they can be ignored.

The container is now initialized and running. You can run `ls` command to check if the rvl-linux, detectron2 and ur5_ws directories are listed. If not, exit the current instance of the container and change the first part of the first --volume argument to look like this:
```bash
--volume="/path/to/your/project/dir:/home/RVLuser" \
```
Rerun the container with:
```bash
./run_docker.sh
```

If you need more terminals inside the container, you can create a new one with:
```bash
docker exec -it rvl_ur5_detectron2 bash
```


### VSCode setup

While the container is running, you can setup the VSCode to work inside the container.
A tutorial on the developing inside a container can be found [here](https://code.visualstudio.com/docs/devcontainers/containers).

Inside the container, you may consider installing some extensions: Python, C/C++, ROS, CMake, Makefile Tools. These aren't necessary, but are helpful. 

You don't need to setup any VSCode environments as they are already set to work with the container's paths (for RVL, ur5_Ws and detectron2). 


## Usage

The current 'user' RVLuser is already root.

### RVL usage

Inside the container, navigate to RVL's root directory:
```bash
cd /home/RVLuser/rvl-linux
```
Build the library with:
```bash
make
```
Configure the necessary .cfg files in the root directory of the library and run the program. For example:
```bash
./build/bin/RVLRecognitionDemo
```

### ROS usage
Inside the container, in one terminal, initialize roscore:
```bash
roscore
```
In another terminal, navigate to the ur5_ws directory:
```bash
cd /home/RVLuser/ur5_ws
```
source the project:
```bash
source devel/setup.bash
```
and run the Python node that takes images from a camera, generates PLY files and runs DDDetector to detect the doors: <!-- (TODO: add the node for detecting state and opening doors) -->
```bash
rosrun ao_manipulation detect_AO_model_node.py
```

### Detectron2 usage
Inside the container, navigate to TensorMask directory:
```bash
cd /home/RVLuser/detectron2/projects/TensorMask
```
Run an example: (needs testing)
```bash
python3 train_net.py --config-file configs/tensormask_R_50_FPN_1x.yaml --eval-only MODEL.WEIGHTS checkpoints/tensormask_R_50_FPN_1x.pkl
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.


## License

[MIT](https://choosealicense.com/licenses/mit/)