execute_process(COMMAND "/home/marco/catkin_ws/src/audio/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/marco/catkin_ws/src/audio/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
