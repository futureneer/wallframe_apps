cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

### Bring in the Qt Libraries ###
find_package(Qt4 REQUIRED)
if(QT_FOUND)
	set (QT_USE_QTWEBKIT TRUE)
	set (QT_USE_QTOPENGL TRUE)
	include(${QT_USE_FILE})
	add_definitions(${QT_DEFINITIONS})
	add_definitions(-DQT_NO_KEYWORDS)
else(QT_FOUND)
	message(FATAL: QT NOT FOUND)
endif(QT_FOUND)

### OpenGL ###
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})

### OpenSceneGraph and OSGQt ###
find_package(OpenSceneGraph COMPONENTS osgWidget osgViewer osgUtil 
  osgGA osgDB osgText osgQt osgParticle osgFX osgManipulator)
if(OpenSceneGraph_FOUND)
  include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
endif(OpenSceneGraph_FOUND)

if(NOT OSGQT_FOUND)
  message(error "Could not find osgQt")
endif()

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Qt MOC Generator (for Q_OBJECT behaviors)
set(_moc ${CMAKE_CURRENT_BINARY_DIR}/${_current_MOC})

## Set your source files here (anything thats has a Q_OBJECT in it)
set(MODULAIR_OSG_TOOLS_SOURCES 	src/osg_object_base.cpp
																src/osg_planar_object.cpp)
set(MODULAIR_OSG_TOOLS_HEADERS 	include/${PROJECT_NAME}/osg_object_base.h
															 	include/${PROJECT_NAME}/osg_planar_object.h
															 	include/${PROJECT_NAME}/state_object.h
															 	include/${PROJECT_NAME}/vector_conversions.h)

## Create MOC wrappers for headers
qt4_wrap_cpp(MODULAIR_OSG_TOOLS_HEADERS_MOC ${MODULAIR_OSG_TOOLS_HEADERS})

rosbuild_add_library(${PROJECT_NAME}	${MODULAIR_OSG_TOOLS_HEADERS}
										${MODULAIR_OSG_TOOLS_SOURCES}
										${MODULAIR_OSG_TOOLS_HEADERS_MOC})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
