cmake_minimum_required(VERSION 3.5.1)
project(alica_ros_flatland_turtle)

set(CMAKE_CXX_STANDARD 17)
##################
## Dependencies ##
##################

set(PKG_DEPS
    roscpp
    alica_ros_proxy
    flatland_msgs
    geometry_msgs
)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

##Yaml
find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface)
find_package(alica_engine REQUIRED)
find_package(alica_turtlesim REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(autodiff REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})
find_package(alica_dynamic_loading REQUIRED)

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    INCLUDE_DIRS include
    DEPENDS autodiff alica_dynamic_loading

)


###########
## Build ##
###########
include_directories(
    include
    ${alica_turtlesim_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    src/base_node.cpp
    src/base.cpp
    src/turtle_ros1_interfaces.cpp
)

target_link_libraries(${PROJECT_NAME}
                        Boost::system
                        Boost::filesystem
                        ${CMAKE_DL_LIBS}
                        ${catkin_LIBRARIES}
                        ${YAML_CPP_LIBRARIES}
                        alica_dynamic_loading
                        autodiff
                        constraintsolver
)

add_subdirectory(libalica-ros-flatland)

