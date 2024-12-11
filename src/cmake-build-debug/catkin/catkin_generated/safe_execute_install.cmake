execute_process(COMMAND "/home/qyd/dx/one_ws/src/cmake-build-debug/catkin/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/qyd/dx/one_ws/src/cmake-build-debug/catkin/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
