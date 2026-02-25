FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update
RUN apt-get install -y wget build-essential git unzip cmake g++ python


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


WORKDIR /
RUN wget https://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.gz
RUN ls
RUN tar -xf boost_1_58_0.tar.gz
WORKDIR /boost_1_58_0
RUN apt-get install -y build-essential g++ python-dev autotools-dev libicu-dev libbz2-dev
RUN ./bootstrap.sh
RUN ./b2
RUN ./b2 install

#OpenNI
RUN apt-get install -y libopenni-dev
#OpenNI2
RUN apt-get install -y libopenni2-dev

# Install PCL (BUILD WITH OPENNI, OPENNI2)
WORKDIR /
RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
RUN tar -xf pcl-1.8.1.tar.gz
RUN sed -i 's/        return (plane_coeff_d_);/        return (\*plane_coeff_d_);/' /pcl-pcl-1.8.1/segmentation/include/pcl/segmentation/plane_coefficient_comparator.h
RUN cd pcl-pcl-1.8.1 && mkdir build
WORKDIR /pcl-pcl-1.8.1/build

RUN cmake -DWITH_OPENNI2=ON -DWITH_OPENNI=ON .. 
RUN make -j4
RUN make install

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/home/RVLuser/rvl-linux/build/lib" >> /etc/bash.bashrc

# RUN wget https://www.baslerweb.com/fp-1668420813/media/downloads/software/pylon_software/pylon_7.2.1.25747_x86_64_debs.tar.gz
# RUN mkdir /pylon_7.2
# RUN tar -xzvf pylon_7.2.1.25747_x86_64_debs.tar.gz -C ./pylon_7.2
# WORKDIR  /pylon_7.2
# # RUN cd ./pylon_7.2
# RUN ls
# RUN apt-get install -y ./codemeter_7.40.4997.501_amd64.deb ./pylon_7.2.1.25747-deb0_amd64.deb
# # RUN dpkg -i codemeter_7.40.4997.501_amd64.deb
# # RUN apt-get update && apt-get install -y 

RUN apt-get update
RUN apt-get -y install python3-pip
RUN pip3 install numpy

# Install pybind 11
RUN pip install pybind11
RUN apt-get update