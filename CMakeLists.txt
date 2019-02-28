cmake_minimum_required(VERSION 3.1.0)
project(pitag)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set( CMAKE_CXX_STANDARD 11 )
list( APPEND CMAKE_CXX_FLAGS "-O3 -fpic" )

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

add_executable(pitag-test
  pitag-test.cpp
)
target_link_libraries(pitag-test
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
)

#############
## Install ##
#############

install( FILES
         ${PROJECT_SOURCE_DIR}/AbstractFiducialModel.h
         ${PROJECT_SOURCE_DIR}/AbstractFiducialParameters.h
         ${PROJECT_SOURCE_DIR}/FiducialModelPi.h
         ${PROJECT_SOURCE_DIR}/FiducialPiParameters.h
         ${PROJECT_SOURCE_DIR}/FiducialDefines.h
         DESTINATION include )

install( TARGETS ${PROJECT_NAME} DESTINATION lib )
