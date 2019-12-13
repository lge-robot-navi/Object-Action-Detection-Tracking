# Install script for directory: /home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osr_msgs/msg" TYPE FILE FILES
    "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg"
    "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg"
    "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osr_msgs/cmake" TYPE FILE FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/build/osr/osr_msgs/catkin_generated/installspace/osr_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/devel/include/osr_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/devel/share/roseus/ros/osr_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/devel/share/common-lisp/ros/osr_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/devel/share/gennodejs/ros/osr_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/devel/lib/python2.7/dist-packages/osr_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/devel/lib/python2.7/dist-packages/osr_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/build/osr/osr_msgs/catkin_generated/installspace/osr_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osr_msgs/cmake" TYPE FILE FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/build/osr/osr_msgs/catkin_generated/installspace/osr_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osr_msgs/cmake" TYPE FILE FILES
    "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/build/osr/osr_msgs/catkin_generated/installspace/osr_msgsConfig.cmake"
    "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/build/osr/osr_msgs/catkin_generated/installspace/osr_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osr_msgs" TYPE FILE FILES "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/package.xml")
endif()

