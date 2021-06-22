cmake_minimum_required(VERSION 3.0.2)
project(ros_timer_factory)

add_compile_options(
  -std=c++14
  -Wall
  -Wextra
  -Wpedantic
  -Werror
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test
    test/main.test
    test/main.cpp
    test/timer.cpp
    test/steady_timer.cpp
    test/wall_timer.cpp
  )

  target_include_directories(${PROJECT_NAME}-test
    PRIVATE
      ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()
