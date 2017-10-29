cmake_minimum_required(VERSION 2.8.3)
project(cube_setup)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Declare things to be passed to dependent projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cube_setup
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(cube_setup
#   src/${PROJECT_NAME}/cube_setup.cpp
# )

## Declare a C++ executable
add_executable(cube 
	src/main.cpp
	src/cube_class.cpp
	src/face_class.cpp
	src/helper.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(cube ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(cube ${catkin_LIBRARIES})

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test test/cube_tests.cpp)
if (CATKIN_ENABLE_TESTING)

	catkin_add_gtest(utest_cube 
		test/cube_tests.cpp
		test/solved_cube_builders.cpp
		test/test_cube1_builders.cpp
		test/test_cube2_builders.cpp
		src/cube_class.cpp
		src/face_class.cpp
		src/helper.cpp
	)

 	target_link_libraries(utest_cube ${catkin_LIBRARIES})

	catkin_add_gtest(utest_face 
		test/face_tests.cpp
		src/face_class.cpp
		src/helper.cpp
	)

 	target_link_libraries(utest_face ${catkin_LIBRARIES})

  	catkin_add_gtest(utest_helper 
		test/helper_tests.cpp
		src/helper.cpp
	)

 	target_link_libraries(utest_helper ${catkin_LIBRARIES})

endif()
