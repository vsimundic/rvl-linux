cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/RVLuser/rvl-linux/modules/RVLCore/Include
  /home/RVLuser/rvl-linux/modules/RVLNN/Include
  /home/RVLuser/rvl-linux/modules/RVLObjectDetection/Include
  /home/RVLuser/rvl-linux/modules/RVLPCSegment/Include
  /home/RVLuser/rvl-linux/modules/RVLRecognition/Include
  /home/RVLuser/rvl-linux/modules/RVLMotion/Include
  /usr/local/include/vtk-7.1 # Add VTK include path
  /home/RVLuser/rvl-linux/3rdparty/RapidXML/include # Add RapidXML include path
  /home/RVLuser/rvl-linux/3rdparty/NanoFlann/Include # Add NanoFlann include path
  /usr/include/eigen3 # Add Eigen include path
)

link_directories(
  /usr/local/lib # Add VTK lib path
  /home/RVLuser/rvl-linux/build/lib
)

# add_executable(${PROJECT_NAME}_node src/test_node.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${VTK_LIBRARIES}
#   RVLMotion  
#   RVLObjectDetection  
#   RVLRecognition
#   RVLObjectDetection  # This has to go twice because RVLRecognition depends on it
#   RVLPCSegment
#   RVLNN
#   RVLCore

#   # /home/RVLuser/rvl-linux/build/lib/libRVLCore.so
#   # /home/RVLuser/rvl-linux/build/lib/libRVLPCSegment.so
#   # /home/RVLuser/rvl-linux/build/lib/libRVLNN.so
#   # /home/RVLuser/rvl-linux/build/lib/libRVLObjectDetection.so
#   # /home/RVLuser/rvl-linux/build/lib/libRVLRecognition.so
#   # /home/RVLuser/rvl-linux/build/lib/libRVLMotion.so

#   opencv_world 
#   vtksys-7.1 
#   vtkCommonCore-7.1 
#   vtkIOGeometry-7.1 
#   vtkCommonExecutionModel-7.1 
#   vtkRenderingCore-7.1 
#   vtkIOCore-7.1 
#   vtkFiltersGeneral-7.1 
#   vtkCommonDataModel-7.1 
#   vtkCommonTransforms-7.1 
#   vtkFiltersSources-7.1 
#   vtkFiltersCore-7.1 
#   vtkCommonComputationalGeometry-7.1 
#   vtkIOPLY-7.1 
#   vtkIOLegacy-7.1 
#   vtkInteractionWidgets-7.1 
#   vtkInteractionStyle-7.1 
#   vtkRenderingAnnotation-7.1 
#   vtkRenderingFreeType-7.1 
#   vtkRenderingContextOpenGL2-7.1 
#   vtkRenderingOpenGL2-7.1 
#   vtkInteractionStyle-7.1 
#   vtkRenderingGL2PSOpenGL2-7.1 
#   vtkRenderingLabel-7.1 
# )