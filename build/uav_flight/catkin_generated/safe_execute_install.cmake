execute_process(COMMAND "/home/isaac/uav_avoidance/build/uav_flight/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/isaac/uav_avoidance/build/uav_flight/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
