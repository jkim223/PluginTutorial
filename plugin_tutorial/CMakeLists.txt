cmake_minimum_required(VERSION 2.8.3)
project(plugin_tutorial)


#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
)

find_package(gazebo REQUIRED)


catkin_package(
   DEPENDS 
    roscpp
    gazebo_ros
    std_msgs
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS}) 

include_directories(
${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)


add_library( 
  #plugin_tutorial src/movel_move.cpp
  model_move SHARED src/movel_move.cpp
)


target_link_libraries( 
   model_move  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
   
)


