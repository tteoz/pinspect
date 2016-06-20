cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(pinspect CXX)

#dependencies
find_package(OpenCV 3.0.0 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.57.0 REQUIRED
  COMPONENTS program_options
             system
             filesystem)

include_directories(
  src/include
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# set(CMAKE_BUILD_TYPE Debug)

add_executable(pinspect
	src/Calibration.cpp
	src/CalibrationSet.cpp
	src/Circlesgrid.cpp
	src/Inspection.cpp
	src/Main.cpp
	src/Parameters.cpp
	src/Pipeline.cpp)

#C++11 standard
SET_PROPERTY (TARGET pinspect PROPERTY CXX_STANDARD 11)
SET_PROPERTY (TARGET pinspect PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(pinspect
  boost_program_options
  boost_system
  boost_filesystem

  opencv_core
  opencv_flann
  opencv_imgproc
  opencv_imgcodecs
  opencv_features2d
  opencv_calib3d
  opencv_highgui

  ${PCL_LIBRARIES}
  )
