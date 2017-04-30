cmake_minimum_required(VERSION 3.1.0)
project(pitag)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)


###########
## Build ##
###########
include_directories(
  common/include
  ${OpenCV_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

set(project_CPP_FILES
  AbstractFiducialModel.cpp
  FiducialModelPi.cpp
  ellipserefine.cpp
)

add_library(${PROJECT_NAME} 
  ${project_CPP_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__")

add_executable(pitag-test
  pitag-test.cpp
)
target_link_libraries(pitag-test
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
)
set_target_properties(pitag-test PROPERTIES COMPILE_FLAGS "-D__LINUX__")
