FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update
RUN apt-get install -y wget build-essential git unzip cmake g++ python


###### RVL ######

RUN apt install -y libeigen3-dev
RUN apt-get install -y cmake-curses-gui
RUN apt-get update
RUN apt-get install -y libgl1-mesa-dev libgl1-mesa-glx xvfb
RUN apt-get update
RUN apt-get install -y gdb libgtk2.0-dev pkg-config
RUN apt-get install -y libhdf5-serial-dev
RUN apt-get update
RUN apt-get install -y libusb-1.0-0-dev libudev-dev
RUN apt-get install -y default-jdk openjdk-11-jdk
RUN apt-get install -y libtiff-dev freeglut3-dev doxygen graphviz

WORKDIR /
# Install VTK
RUN wget https://gitlab.kitware.com/vtk/vtk/-/archive/v7.1.1/vtk-v7.1.1.tar.gz
RUN tar -xf vtk-v7.1.1.tar.gz
RUN apt install -y libgl1-mesa-dev libxt-dev
RUN cd vtk-v7.1.1 && mkdir build && cd build && cmake -DBUILD_TESTING=OFF .. && make -j$(nproc) && make install

# Install OpenCV
RUN wget https://github.com/opencv/opencv/archive/3.4.16.zip
# RUN mkdir /opencv-3.4.16
RUN unzip 3.4.16.zip
RUN git clone --depth 1 --branch '3.4.16' https://github.com/opencv/opencv_contrib.git
RUN ls /opencv_contrib
WORKDIR /opencv-3.4.16
RUN mkdir build && cd build && cmake -DOPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules -DWITH_EIGEN=ON -DWITH_VTK=ON -DBUILD_opencv_world=ON .. && make -j$(nproc) && make install



WORKDIR /
RUN git clone --depth 1 --branch '1.8.4' https://github.com/flann-lib/flann.git
RUN cd flann && touch src/cpp/empty.cpp && sed -e '/add_library(flann_cpp SHARED/ s/""/empty.cpp/' \
    -e '/add_library(flann SHARED/ s/""/empty.cpp/' \
    -i src/cpp/CMakeLists.txt
RUN cd flann && mkdir build && cd build && cmake .. && make -j$(nproc) && make install


# WORKDIR /
# RUN wget https://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.gz
# RUN ls
# RUN tar -xf boost_1_58_0.tar.gz
# WORKDIR /boost_1_58_0
# RUN apt-get install -y build-essential g++ python-dev autotools-dev libicu-dev libbz2-dev
# RUN ./bootstrap.sh --prefix=/boost_1_58_0
# RUN ./b2
# RUN ./b2 install


#OpenNI
RUN apt-get install -y libopenni-dev
#OpenNI2
RUN apt-get install -y libopenni2-dev

# # Install PCL (BUILD WITH OPENNI, OPENNI2)
# WORKDIR /
# RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
# RUN tar -xf pcl-1.8.1.tar.gz
# RUN sed -i 's/        return (plane_coeff_d_);/        return (\*plane_coeff_d_);/' /pcl-pcl-1.8.1/segmentation/include/pcl/segmentation/plane_coefficient_comparator.h
# ENV BOOST_ROOT=/boost_1_58_0
# RUN echo $BOOST_ROOT
# RUN cd pcl-pcl-1.8.1 && mkdir build
# WORKDIR /pcl-pcl-1.8.1/build

# RUN cmake -DWITH_OPENNI2=ON -DWITH_OPENNI=ON .. 
# RUN make -j8
# RUN make install

# RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/home/RVLuser/rvl-linux/build/lib:/boost_1_58_0/lib" >> /etc/bash.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/home/RVLuser/rvl-linux/build/lib" >> /etc/bash.bashrc

RUN apt-get update
RUN apt-get -y install python3-pip
RUN pip3 install numpy

# Install pybind 11
RUN pip3 install pybind11
RUN apt-get update

ENV PYTHONPATH "${PYTHONPATH}:/home/RVLuser/rvl-linux/python/build/lib"



###### DETECTRON2 ######
WORKDIR /home/RVLuser
COPY detectron2/ /home/RVLuser/detectron2
RUN apt-get update && apt-get install -y \
    python3-opencv ca-certificates python3-dev git wget sudo ninja-build
# RUN ln -sv /usr/bin/python3 /usr/bin/python

RUN wget https://bootstrap.pypa.io/pip/3.6/get-pip.py && \
    python3 get-pip.py --user && \
    rm get-pip.py

# install dependencies
# See https://pytorch.org/ for other options if you use a different version of CUDA
RUN pip3 install tensorboard onnx   # cmake from apt-get is too old
RUN pip3 install torch==1.10 torchvision==0.11.1 -f https://download.pytorch.org/whl/cu111/torch_stable.html

RUN pip3 install 'git+https://github.com/facebookresearch/fvcore'
# install detectron2
# RUN git clone https://github.com/facebookresearch/detectron2 detectron2_repo
# set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build detectron2 for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"

RUN apt-get install python3-setuptools
RUN pip3 install cython

RUN apt-get update
RUN pip3 install matplotlib==3.7.2
RUN pip3 install omegaconf==2.3.0

WORKDIR /home/RVLuser/detectron2
# RUN pip3 install -e detectron2
RUN python3 setup.py build develop
ENV FVCORE_CACHE="/tmp"


# RUN pip3 install -e /home/RVLuser/detectron2/projects/TensorMask
WORKDIR /home/RVLuser/detectron2/projects/TensorMask
RUN python3 setup.py build develop


###### ROS-UR5 ######

RUN apt-get update
# RUN apt-get -y install python3-pip

RUN apt-get update
RUN apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install -y ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools

RUN apt install python3-rosdep
RUN apt-get install -y ros-noetic-realsense2-camera
RUN apt-get install -y ros-noetic-openni-launch
RUN apt-get install -y ros-noetic-openni2-launch
RUN apt-get install -y ros-noetic-rosbash
RUN apt-get install -y ros-noetic-ros-control
RUN apt-get install -y ros-noetic-soem
RUN apt-get install -y ros-noetic-moveit
RUN apt-get install -y ros-noetic-trac-ik
RUN apt-get install -y ros-noetic-industrial-core ros-noetic-ros-industrial-cmake-boilerplate ros-noetic-socketcan-interface ros-noetic-industrial-robot-status-interface ros-noetic-ros-controllers ros-noetic-scaled-joint-trajectory-controller ros-noetic-speed-scaling-interface ros-noetic-speed-scaling-state-controller ros-noetic-ur-msgs ros-noetic-pass-through-controllers ros-noetic-ur-client-library

RUN pip3 install pymodbus --upgrade
RUN pip3 install ur-rtde
RUN pip3 install pyyaml

# RUN mkdir -p /home/RVLuser/ur5_ws/src
# COPY ur5_ws/src/ /home/RVLuser/ur5_ws/src/
# RUN ls /opt/ros/noetic
# RUN rosdep init
# RUN rosdep update
# WORKDIR /home/RVLuser/ur5_ws
# RUN rosdep install --from-paths src --ignore-src -r --rosdistro noetic -y

# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# ENV PYTHONPATH="${PYTHONPATH}:/home/RVLuser/detectron2/detectron2"

RUN ln -sf /usr/bin/python3 /usr/bin/python

RUN pip3 install open3d
RUN pip3 install Pillow==9.0.0

# Install aruco lib
WORKDIR /
RUN wget https://sourceforge.net/projects/aruco/files/latest/download -O aruco-3.1.12.zip
RUN unzip aruco-3.1.12.zip
RUN cd aruco-3.1.12 && mkdir build && cd build && cmake .. && make -j$(nproc) && make install
RUN sh -c 'echo "/usr/local/include/aruco/" > /etc/ld.so.conf.d/aruco.conf'
RUN ldconfig
RUN pip3 install aruco
RUN pip3 install pytransform3d


ENV PYTHONPATH="${PYTHONPATH}:/home/RVLuser/rvl-linux/modules/RVLPY"
ENV PYTHONPATH="${PYTHONPATH}:/home/RVLuser/rvl-linux/python"
