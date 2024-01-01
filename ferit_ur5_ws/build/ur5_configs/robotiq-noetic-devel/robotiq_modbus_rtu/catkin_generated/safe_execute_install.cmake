execute_process(COMMAND "/home/RVLuser/ferit_ur5_ws/build/ur5_configs/robotiq-noetic-devel/robotiq_modbus_rtu/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/RVLuser/ferit_ur5_ws/build/ur5_configs/robotiq-noetic-devel/robotiq_modbus_rtu/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
