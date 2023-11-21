# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

# Default target executed when no arguments are given to make.

#default_target: all

#.PHONY : default_target

#=============================================================================

# Make build directories if they do not exist already.

BUILD_DIRS = build build/bin build/lib build/obj
$(shell mkdir -p $(BUILD_DIRS))
MKDIR_BUILDS = @mkdir -p $(@D)


# Set paths.

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/RVLuser/rvl-linux

# OpenCV include directory.
OPENCV_INCLUDE_DIR = /usr/local/include

# OpenCV lib directory.
OPENCV_LIB_DIR = /usr/local/lib

# VTK include directory.
VTK_INCLUDE_DIR = /usr/local/include/vtk-7.1

# VTK lib directory.
VTK_LIB_DIR = /usr/local/lib

# # PCL directory.
# PCL_DIR = /usr/local

# # PCL lib directory.
# PCL_LIB_DIR = $(PCL_DIR)/lib

# FLANN include directory.
FLANN_INCLUDE_DIR = /usr/local/include/flann

# FLANN lib directory.
FLANN_LIB_DIR = /usr/local/lib

# Eigen include directory.
EIGEN_INCLUDE_DIR = /usr/include/eigen3

# RapidXML include directory.
RAPIDXML_INCLUDE_DIR = /home/RVLuser/rvl-linux/3rdparty/RapidXML/include

# NanoFLANN include directory.
NANOFLANN_INCLUDE_DIR = /home/RVLuser/rvl-linux/3rdparty/NanoFlann/Include


#=============================================================================
# Set environment variables for the build.
# export OPENNI2_INCLUDE_DIRS:PATH=/usr/include/openni2
# export OPENNI2_LIBRARY:FILEPATH=/usr/lib/libOpenNI2.so

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = $(CMAKE_SOURCE_DIR)/build
RVLPY_DIR = $(CMAKE_SOURCE_DIR)/python
CMAKE_PYTHON_BINARY_DIR = $(RVLPY_DIR)/build
CMAKE_APP_DIR = $(CMAKE_BINARY_DIR)/bin

# # PCL includes.
# PCL_INCLUDES = -I$(PCL_DIR)/include/pcl-1.8 -I$(EIGEN_INCLUDE_DIR) -I$(FLANN_INCLUDE_DIR) -I/usr/include/openni2

# External includes.
EXTERNAL_INCLUDES = -I$(OPENCV_INCLUDE_DIR) -I$(VTK_INCLUDE_DIR) -I$(EIGEN_INCLUDE_DIR) -I$(RAPIDXML_INCLUDE_DIR)

# External lib directories.
# EXTERNAL_LIB_DIR = -L$(OPENCV_LIB_DIR) -L$(VTK_LIB_DIR) -L$(PCL_LIB_DIR) -L$(FLANN_LIB_DIR) # -L/usr/lib/x86_64-linux-gnu
EXTERNAL_LIB_DIR = -L$(OPENCV_LIB_DIR) -L$(VTK_LIB_DIR) -L$(FLANN_LIB_DIR) # -L/usr/lib/x86_64-linux-gnu

# RVL lib directory.
RVL_LIB_DIR = $(CMAKE_BINARY_DIR)/lib
RVLPY_LIB_DIR = $(CMAKE_PYTHON_BINARY_DIR)/lib

# External libraries.
EXTERNAL_LIBS = -lopencv_world \
 -lvtksys-7.1 \
 -lvtkCommonCore-7.1 \
 -lvtkIOGeometry-7.1 \
 -lvtkCommonExecutionModel-7.1 \
 -lvtkRenderingCore-7.1 \
 -lvtkIOCore-7.1 \
 -lvtkFiltersGeneral-7.1 \
 -lvtkCommonDataModel-7.1 \
 -lvtkCommonTransforms-7.1 \
 -lvtkFiltersSources-7.1 \
 -lvtkFiltersCore-7.1 \
 -lvtkCommonComputationalGeometry-7.1 \
 -lvtkIOPLY-7.1 \
 -lvtkIOLegacy-7.1 \
 -lvtkInteractionWidgets-7.1 \
 -lvtkInteractionStyle-7.1 \
 -lvtkRenderingAnnotation-7.1 \
 -lvtkRenderingFreeType-7.1 \
 -lvtkRenderingContextOpenGL2-7.1 \
 -lvtkRenderingOpenGL2-7.1 \
 -lvtkInteractionStyle-7.1 \
 -lvtkRenderingGL2PSOpenGL2-7.1 \
 -lvtkRenderingLabel-7.1 

 # PCL libraries
PCL_LIBS = -lboost_system \
 -lpcl_common \
 -lpcl_search \
 -lpcl_surface \
 -lpcl_filters \
 -lpcl_io \
 -lpcl_features
#  -lpcl_common \
#  -lpcl_search \
#  -lpcl_surface \
#  -lpcl_filters \
#  -lpcl_io \
#  -lpcl_features

# -lflann_cpp_s \
# -lpcl_search \
# -llz4

# Python

# PYTHON_SUFFIX for a particular python installation can be obtained by command: python3-config --extension-suffix
PYTHON_SUFFIX := .cpython-38-x86_64-linux-gnu.so

# PYBIND11_INCLUDES for a particular pybind11 installation can be obtained by command: python3 -m pybind11 --includes
PYBIND11_INCLUDES := -I/usr/include/python3.8 -I/usr/local/lib/python3.8/dist-packages/pybind11/include

RVLPY_SRC_DIR := $(RVLPY_DIR)/src



#=============================================================================
# RVLCore library

# Object file directory.
RVLCORE_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLCore

# Source directory.
RVLCORE_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLCore

# Include directory.
RVLCORE_INCLUDE_DIR = $(RVLCORE_SRC_DIR)/Include

# Includes.
RVLCORE_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Output library.
RVLCORE_LIB := $(RVL_LIB_DIR)/libRVLCore.a
RVLCORE_DLIB := $(RVL_LIB_DIR)/libRVLCore.so


#=============================================================================
# RVLPCSegment library

# Object file directory.
RVLPCSEGMENT_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLPCSegment

# Source directory.
RVLPCSEGMENT_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLPCSegment

# Include directory.
RVLPCSEGMENT_INCLUDE_DIR = $(RVLPCSEGMENT_SRC_DIR)/Include

# Includes.
RVLPCSEGMENT_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Output library.
RVLPCSEGMENT_LIB := $(RVL_LIB_DIR)/libRVLPCSegment.a
RVLPCSEGMENT_DLIB := $(RVL_LIB_DIR)/libRVLPCSegment.so


#=============================================================================
# RVLPCLTools library

# Object file directory.
RVLPCLTOOLS_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLPCLTools

# Source directory.
RVLPCLTOOLS_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLPCLTools

# Include directory.
RVLPCLTOOLS_INCLUDE_DIR = $(RVLPCLTOOLS_SRC_DIR)/Include

# Includes.
RVLPCLTOOLS_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLPCLTOOLS_INCLUDE_DIR) $(EXTERNAL_INCLUDES) $(PCL_INCLUDES) 

# Output library.
RVLPCLTOOLS_LIB := $(RVL_LIB_DIR)/libRVLPCLTools.a
RVLPCLTOOLS_DLIB := $(RVL_LIB_DIR)/libRVLPCLTools.so


#=============================================================================
# RVLObjectDetection library

# Object file directory.
RVLOBJECTDETECTION_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLObjectDetection

# Source directory.
RVLOBJECTDETECTION_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLObjectDetection

# Include directory.
RVLOBJECTDETECTION_INCLUDE_DIR = $(RVLOBJECTDETECTION_SRC_DIR)/Include

# Includes.
RVLOBJECTDETECTION_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLOBJECTDETECTION_INCLUDE_DIR) -I$(RVLRECOGNITION_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Output library.
RVLOBJECTDETECTION_LIB := $(RVL_LIB_DIR)/libRVLObjectDetection.a
RVLOBJECTDETECTION_DLIB := $(RVL_LIB_DIR)/libRVLObjectDetection.so

#=============================================================================
# RVLRecognition library

# Object file directory.
RVLRECOGNITION_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLRecognition

# Source directory.
RVLRECOGNITION_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLRecognition

# Include directory.
RVLRECOGNITION_INCLUDE_DIR = $(RVLRECOGNITION_SRC_DIR)/Include

# Includes.
RVLRECOGNITION_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLOBJECTDETECTION_INCLUDE_DIR) -I$(RVLRECOGNITION_INCLUDE_DIR) $(EXTERNAL_INCLUDES) -I$(NANOFLANN_INCLUDE_DIR)
# RVLRECOGNITION_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLRECOGNITION_INCLUDE_DIR) $(EXTERNAL_INCLUDES) -I$(NANOFLANN_INCLUDE_DIR)

# Output library.
RVLRECOGNITION_LIB := $(RVL_LIB_DIR)/libRVLRecognition.a
RVLRECOGNITION_DLIB := $(RVL_LIB_DIR)/libRVLRecognition.so


#=============================================================================
# RVLLocalization library

# Object file directory.
RVLLOCALIZATION_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLLocalization

# Source directory.
RVLLOCALIZATION_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLLocalization

# Include directory.
RVLLOCALIZATION_INCLUDE_DIR = $(RVLLOCALIZATION_SRC_DIR)/Include

# Includes.
RVLLOCALIZATION_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLLOCALIZATION_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Output library.
RVLLOCALIZATION_LIB := $(RVL_LIB_DIR)/libRVLRVLLocalization.a


#=============================================================================
# RVLNN library

# Object file directory.
RVLNN_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLNN

# Source directory.
RVLNN_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLNN

# Include directory.
RVLNN_INCLUDE_DIR = $(RVLNN_SRC_DIR)/Include

# Includes.
RVLNN_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLNN_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Output library.
RVLNN_LIB := $(RVL_LIB_DIR)/libRVLNN.a
RVLNN_DLIB := $(RVL_LIB_DIR)/libRVLNN.so


#=============================================================================
# RVLMesher

# Object file directory.
RVLMESHER_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLMesher

# Source directory.
RVLMESHER_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLMesher

# Library directories.
#RVLMESHER_LIB_DIR = -L$()

# Includes.
RVLMESHER_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLPCLTOOLS_INCLUDE_DIR) $(EXTERNAL_INCLUDES) $(PCL_INCLUDES)

# Libraries.
RVLMESHER_LIBS := $(RVLCORE_LIB) $(RVLPCSEGMENT_LIB) $(RVLPCLTOOLS_LIB)
RVLMESHER_LIBS_ := -lRVLCore -lRVLPCSegment -lRVLPCLTools $(EXTERNAL_LIBS)

# Output app.
RVLMESHER_APP := $(CMAKE_BINARY_DIR)/bin/RVLMesher


#=============================================================================
# RVLRecognitionDemo

# Object file directory.
RVLRECOGNITIONDEMO_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLRecognitionDemo

# Source directory.
RVLRECOGNITIONDEMO_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLRecognitionDemo

# Library directories.
#RVLMESHER_LIB_DIR = -L$()

# Includes.
RVLRECOGNITIONDEMO_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLOBJECTDETECTION_INCLUDE_DIR) -I$(RVLRECOGNITION_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Libraries.
RVLRECOGNITIONDEMO_LIBS := $(RVLCORE_LIB) $(RVLPCSEGMENT_LIB)
RVLRECOGNITIONDEMO_LIBS_ := -lRVLCore -lRVLPCSegment $(EXTERNAL_LIBS)

# Output app.
RVLRECOGNITIONDEMO_APP := $(CMAKE_BINARY_DIR)/bin/RVLRecognitionDemo


#=============================================================================
# RVLNNDemo

# Object file directory.
RVLNNDEMO_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLNNDemo

# Source directory.
RVLNNDEMO_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLNNDemo

# Library directories.
#RVLMESHER_LIB_DIR = -L$()

# Includes.
RVLNNDEMO_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLNN_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Libraries.
RVLNNDEMO_LIBS := $(RVLCORE_LIB) $(RVLPCSEGMENT_LIB)
RVLNNDEMO_LIBS_ := -lRVLCore -lRVLPCSegment $(EXTERNAL_LIBS)

# Output app.
RVLNNDEMO_APP := $(CMAKE_BINARY_DIR)/bin/RVLNNDemo


#=============================================================================
# RVLPYNN

# Includes.
RVLPYNN_INCLUDES = $(PYBIND11_INCLUDES) $(RVLNN_INCLUDES)

# Libraries.
RVLPYNN_LIBS := -lRVLCore -lRVLPCSegment -lRVLNN $(EXTERNAL_LIBS)


#=============================================================================
# RVLPYRGBD2PLY

# Includes.
RVLPYRGBD2PLY_INCLUDES = $(PYBIND11_INCLUDES) $(RVLPCSEGMENT_INCLUDES)

# Libraries.
RVLPYRGBD2PLY_LIBS := -lRVLPCSegment -lRVLCore $(EXTERNAL_LIBS)


#=============================================================================
# RVLPYDDDetector

# Includes.
RVLPYDDDETECTOR_INCLUDES = $(PYBIND11_INCLUDES) $(RVLRECOGNITION_INCLUDES)

# Libraries.
RVLPYDDDETECTOR_LIBS := -lRVLRecognition -lRVLPCSegment -lRVLCore $(EXTERNAL_LIBS)


#=============================================================================
# RVLMotion library

# Object file directory.
RVLMOTION_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLMotion

# Source directory.
RVLMOTION_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLMotion

# Include directory.
RVLMOTION_INCLUDE_DIR = $(RVLMOTION_SRC_DIR)/Include

# Includes.
RVLMOTION_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLMOTION_INCLUDE_DIR) -I$(RVLOBJECTDETECTION_INCLUDE_DIR) -I$(RVLRECOGNITION_INCLUDE_DIR) $(EXTERNAL_INCLUDES) -I$(NANOFLANN_INCLUDE_DIR)
# RVLMOTION_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLMOTION_INCLUDE_DIR) $(EXTERNAL_INCLUDES) -I$(NANOFLANN_INCLUDE_DIR)

# Output library.
RVLMOTION_LIB := $(RVL_LIB_DIR)/libRVLMotion.a
RVLMOTION_DLIB := $(RVL_LIB_DIR)/libRVLMotion.so


#=============================================================================
# RVLMotionDemo

# Object file directory.
RVLMOTIONDEMO_OBJ_DIR = $(CMAKE_BINARY_DIR)/obj/RVLMotionDemo

# Source directory.
RVLMOTIONDEMO_SRC_DIR = $(CMAKE_SOURCE_DIR)/modules/RVLMotionDemo

# Library directories.
#RVLMESHER_LIB_DIR = -L$()

# Includes.
RVLMOTIONDEMO_INCLUDES = -I$(RVLCORE_INCLUDE_DIR) -I$(RVLPCSEGMENT_INCLUDE_DIR) -I$(RVLOBJECTDETECTION_INCLUDE_DIR) -I$(RVLRECOGNITION_INCLUDE_DIR) -I$(RVLMOTION_INCLUDE_DIR) $(EXTERNAL_INCLUDES)

# Libraries.
RVLMOTIONDEMO_LIBS := $(RVLCORE_LIB) $(RVLPCSEGMENT_LIB)
RVLMOTIONDEMO_LIBS_ := -lRVLCore -lRVLPCSegment $(EXTERNAL_LIBS)

# Output app.
RVLMOTIONDEMO_APP := $(CMAKE_BINARY_DIR)/bin/RVLMotionDemo


#=============================================================================


# Collect.

# Output libraries.
#LIBS := $(RVLCORE_LIB) $(RVLPCSEGMENT_LIB) $(RVLPCLTOOLS_LIB) $(RVLLOCALIZATION_LIB)
#LIBS := $(RVLCORE_LIB) $(RVLCORE_DLIB) \
 $(RVLPCSEGMENT_LIB) $(RVLPCSEGMENT_DLIB) \
 $(RVLPCLTOOLS_LIB) $(RVLPCLTOOLS_DLIB) \
 $(RVLOBJECTDETECTION_LIB) \
 $(RVLRECOGNITION_LIB) \
 $(RVLNN_LIB) $(RVLNN_DLIB) \
 $(RVLPY_LIB_DIR)/RVLPYNN$(PYTHON_SUFFIX) \
 $(RVLPY_LIB_DIR)/RVLPYVisualizer$(PYTHON_SUFFIX) \
 $(RVLPY_LIB_DIR)/RVLPYRGBD2PLY$(PYTHON_SUFFIX) \
 $(RVLPY_LIB_DIR)/RVLPYDDDetector$(PYTHON_SUFFIX)

LIBS := $(RVLCORE_LIB) $(RVLCORE_DLIB) \
 $(RVLPCSEGMENT_LIB) $(RVLPCSEGMENT_DLIB) \
 $(RVLOBJECTDETECTION_LIB) \
 $(RVLRECOGNITION_LIB) \
 $(RVLMOTION_LIB) \
 $(RVLNN_LIB) $(RVLNN_DLIB) \
 $(RVLPY_LIB_DIR)/RVLPYNN$(PYTHON_SUFFIX) \
 $(RVLPY_LIB_DIR)/RVLPYVisualizer$(PYTHON_SUFFIX) \
 $(RVLPY_LIB_DIR)/RVLPYRGBD2PLY$(PYTHON_SUFFIX) \
 $(RVLPY_LIB_DIR)/RVLPYDDDetector$(PYTHON_SUFFIX)

# Apps.
# APPS := $(RVLMESHER_APP) $(RVLRECOGNITIONDEMO_APP) $(RVLNNDEMO_APP)
APPS := $(RVLMOTIONDEMO_APP) $(RVLRECOGNITIONDEMO_APP)

# Linker flags.
LINKERFLAG = -lm

#=============================================================================
# Main part.

# Compiler.
CC = gcc -std=c++11 -fPIC
PYCC = c++ -shared -std=c++11 -fPIC

# Linker.
LINK = g++

.PHONY = all clean

all: $(LIBS) $(APPS)

#%: %.o
#        @echo "Checking.."
#        ${CC} ${LINKERFLAG} $< -o $@

#${CC} -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) ${LINKERFLAG} -lRVLCore -lRVLPCSegment -lRVLPCLTools $< -o $@
#	${LINK} $(RVL_LIB_DIR)/libRVLCore.a $< -o $@
#	${LINK} $(RVLCORE_OBJ_DIR)/RVLMem.o $< -o $@




# RVLMesher

$(CMAKE_APP_DIR)/RVLMesher: $(RVLMESHER_OBJ_DIR)/RVLMesher.o
	@echo "Linking RVLMesher..."
	${LINK} -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ -lRVLCore -lRVLPCSegment $(EXTERNAL_LIBS) -ggdb
	@echo "RVLMesher built."

$(RVLMESHER_OBJ_DIR)/%.o: $(RVLMESHER_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLMESHER_INCLUDES) -c $< -o $@ -ggdb

# RVLRecognitionDemo

$(CMAKE_APP_DIR)/RVLRecognitionDemo: $(RVLRECOGNITIONDEMO_OBJ_DIR)/RVLRecognitionDemo.o
	@echo "Linking RVLRecognitionDemo..."
	${LINK} -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ -lRVLRecognition -lRVLObjectDetection -lRVLPCSegment -lRVLCore $(EXTERNAL_LIBS) -ggdb
	@echo "RVLRecognitionDemo built."

$(RVLRECOGNITIONDEMO_OBJ_DIR)/%.o: $(RVLRECOGNITIONDEMO_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLRECOGNITIONDEMO_INCLUDES) -c $< -o $@ -ggdb


# RVLMotionDemo

$(CMAKE_APP_DIR)/RVLMotionDemo: $(RVLMOTIONDEMO_OBJ_DIR)/RVLMotionDemo.o
	@echo "Linking RVLMotionDemo..."
	${LINK} -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ -lRVLMotion -lRVLObjectDetection -lRVLRecognition -lRVLObjectDetection -lRVLRecognition -lRVLPCSegment -lRVLCore $(EXTERNAL_LIBS) -ggdb -fpermissive
	@echo "RVLMotionDemo built."

$(RVLMOTIONDEMO_OBJ_DIR)/%.o: $(RVLMOTIONDEMO_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLMOTIONDEMO_INCLUDES) -c $< -o $@ -ggdb -fpermissive


# RVLNNDemo

$(CMAKE_APP_DIR)/RVLNNDemo: $(RVLNNDEMO_OBJ_DIR)/RVLNNDemo.o
	@echo "Linking RVLNNDemo..."
	${LINK} -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ -lRVLNN -lRVLPCSegment -lRVLCore $(EXTERNAL_LIBS) -ggdb
	@echo "RVLNNDemo built."

$(RVLNNDEMO_OBJ_DIR)/%.o: $(RVLNNDEMO_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLNNDEMO_INCLUDES) -c $< -o $@ -ggdb

# RVLPYNN

$(RVLPY_LIB_DIR)/RVLPYNN$(PYTHON_SUFFIX): $(RVLPY_SRC_DIR)/RVLPYNN.cpp
	@echo "Building RVLPYNN..."
	$(MKDIR_BUILDS)
	${PYCC} $(RVLPYNN_INCLUDES) -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ $(RVLPYNN_LIBS) -ggdb
	@echo "RVLPYNN built."

# RVLPYVisualizer

$(RVLPY_LIB_DIR)/RVLPYVisualizer$(PYTHON_SUFFIX): $(RVLPY_SRC_DIR)/RVLPYVisualizer.cpp
	@echo "Building RVLPYVisualizer..."
	$(MKDIR_BUILDS)
	${PYCC} $(RVLPYNN_INCLUDES) -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ $(RVLPYNN_LIBS) -ggdb
	@echo "RVLPYVisualizer built."

# RVLPYRGBD2PLY

$(RVLPY_LIB_DIR)/RVLPYRGBD2PLY$(PYTHON_SUFFIX): $(RVLPY_SRC_DIR)/RVLPYRGBD2PLY.cpp
	@echo "Building RVLPYRGBD2PLY..."
	$(MKDIR_BUILDS)
	${PYCC} $(RVLPYRGBD2PLY_INCLUDES) -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ $(RVLPYRGBD2PLY_LIBS) -ggdb
	@echo "RVLPYRGBD2PLY built."

# RVLPYDDDetector

$(RVLPY_LIB_DIR)/RVLPYDDDetector$(PYTHON_SUFFIX): $(RVLPY_SRC_DIR)/RVLPYDDDetector.cpp
	@echo "Building RVLPYDDDetector..."
	$(MKDIR_BUILDS)
	${PYCC} $(RVLPYDDDETECTOR_INCLUDES) -L$(RVL_LIB_DIR) $(EXTERNAL_LIB_DIR) $< -o $@ $(RVLPYDDDETECTOR_LIBS) -ggdb
	@echo "RVLPYDDDetector built."

# RVLCore

$(RVLCORE_LIB): $(patsubst $(RVLCORE_SRC_DIR)/%.cpp, $(RVLCORE_OBJ_DIR)/%.o, $(wildcard $(RVLCORE_SRC_DIR)/*.cpp))
	@echo "Creating RVLCore static library..."
	ar rcs $@ $^
	@echo "RVLCore static library created."

$(RVLCORE_DLIB): $(patsubst $(RVLCORE_SRC_DIR)/%.cpp, $(RVLCORE_OBJ_DIR)/%.o, $(wildcard $(RVLCORE_SRC_DIR)/*.cpp))
	@echo "Creating RVLCore dynamic library..."
	${CC} -shared $^ -o $@ -ggdb
	@echo "RVLCore dynamic library created."

$(RVLCORE_OBJ_DIR)/%.o: $(RVLCORE_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLCORE_INCLUDES) -c $< -o $@ -ggdb

# RVLPCLTools

$(RVLPCLTOOLS_LIB): $(patsubst $(RVLPCLTOOLS_SRC_DIR)/%.cpp, $(RVLPCLTOOLS_OBJ_DIR)/%.o, $(wildcard $(RVLPCLTOOLS_SRC_DIR)/*.cpp))
	@echo "Creating RVLPCLTools library..."
	ar rcs $@ $^
	@echo "RVLPCLTools library created."

$(RVLPCLTOOLS_DLIB): $(patsubst $(RVLPCLTOOLS_SRC_DIR)/%.cpp, $(RVLPCLTOOLS_OBJ_DIR)/%.o, $(wildcard $(RVLPCLTOOLS_SRC_DIR)/*.cpp))
	@echo "Creating RVLPCLTools dynamic library..."
	${CC} -shared $^ -o $@ -ggdb
	@echo "RVLPCLTools dynamic library created."

$(RVLPCLTOOLS_OBJ_DIR)/%.o: $(RVLPCLTOOLS_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLPCLTOOLS_INCLUDES) -c $< -o $@ -ggdb

# RVLPCSegment

$(RVLPCSEGMENT_LIB): $(patsubst $(RVLPCSEGMENT_SRC_DIR)/%.cpp, $(RVLPCSEGMENT_OBJ_DIR)/%.o, $(wildcard $(RVLPCSEGMENT_SRC_DIR)/*.cpp))
	@echo "Creating RVLPCSegment static library..."
	ar rcs $@ $^
	@echo "RVLPCSegment static library created."

$(RVLPCSEGMENT_DLIB): $(patsubst $(RVLPCSEGMENT_SRC_DIR)/%.cpp, $(RVLPCSEGMENT_OBJ_DIR)/%.o, $(wildcard $(RVLPCSEGMENT_SRC_DIR)/*.cpp))
	@echo "Creating RVLPCSegment dynamic library..."
	${CC} -shared $^ -o $@  -ggdb
	@echo "RVLPCSegment dynamic library created."

$(RVLPCSEGMENT_OBJ_DIR)/%.o: $(RVLPCSEGMENT_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLPCSEGMENT_INCLUDES) -c $< -o $@ -ggdb

# RVLObjectDetection

$(RVLOBJECTDETECTION_LIB): $(patsubst $(RVLOBJECTDETECTION_SRC_DIR)/%.cpp, $(RVLOBJECTDETECTION_OBJ_DIR)/%.o, $(wildcard $(RVLOBJECTDETECTION_SRC_DIR)/*.cpp))
	@echo "Creating RVLObjectDetection library..."
	ar rcs $@ $^
	@echo "RVLObjectDetection library created."

$(RVLOBJECTDETECTION_OBJ_DIR)/%.o: $(RVLOBJECTDETECTION_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLOBJECTDETECTION_INCLUDES) -c $< -o $@ -ggdb

# RVLRecognition

$(RVLRECOGNITION_LIB): $(patsubst $(RVLRECOGNITION_SRC_DIR)/%.cpp, $(RVLRECOGNITION_OBJ_DIR)/%.o, $(wildcard $(RVLRECOGNITION_SRC_DIR)/*.cpp))
	@echo "Creating RVLRecognition library..."
	ar rcs $@ $^
	@echo "RVLRecognition library created."

$(RVLRECOGNITION_OBJ_DIR)/%.o: $(RVLRECOGNITION_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLRECOGNITION_INCLUDES) -c $< -o $@ -ggdb


# RVLMotion

$(RVLMOTION_LIB): $(patsubst $(RVLMOTION_SRC_DIR)/%.cpp, $(RVLMOTION_OBJ_DIR)/%.o, $(wildcard $(RVLMOTION_SRC_DIR)/*.cpp))
	@echo "Creating RVLMotion library..."
	ar rcs $@ $^
	@echo "RVLMotion library created."

$(RVLMOTION_OBJ_DIR)/%.o: $(RVLMOTION_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLMOTION_INCLUDES) -c $< -o $@ -ggdb -fpermissive


# RVLNN

$(RVLNN_LIB): $(patsubst $(RVLNN_SRC_DIR)/%.cpp, $(RVLNN_OBJ_DIR)/%.o, $(wildcard $(RVLNN_SRC_DIR)/*.cpp))
	@echo "Creating RVLNN static library..."
	ar rcs $@ $^
	@echo "RVLNN static library created."

$(RVLNN_DLIB): $(patsubst $(RVLNN_SRC_DIR)/%.cpp, $(RVLNN_OBJ_DIR)/%.o, $(wildcard $(RVLNN_SRC_DIR)/*.cpp))
	@echo "Creating RVLNN dynamic library..."
	${CC} -shared $^ -o $@ 
	@echo "RVLNN dynamic library created."

$(RVLNN_OBJ_DIR)/%.o: $(RVLNN_SRC_DIR)/%.cpp
	$(MKDIR_BUILDS)
	${CC} $(RVLNN_INCLUDES) -c $< -o $@

#$(RVLLOCALIZATION_LIB): $(patsubst $(RVLLOCALIZATION_SRC_DIR)/%.cpp, $(RVLLOCALIZATION_OBJ_DIR)/%.o, $(wildcard $(RVLLOCALIZATION_SRC_DIR)/*.cpp))
#	@echo "Creating RVLLocalization library..."
#	ar rcs $@ $^
#	@echo "RVLLocalization library created."

#$(RVLLOCALIZATION_OBJ_DIR)/%.o: $(RVLLOCALIZATION_SRC_DIR)/%.cpp
# 	$(MKDIR_BUILDS)
#	${CC} $(RVLLOCALIZATION_INCLUDES) -c $< -o $@

clean:
	@echo "Cleaning up..."
	rm -rf build/*
	${BINS}

