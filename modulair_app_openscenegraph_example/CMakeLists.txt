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
set(OSG_APP_SOURCES src/osg_app.cpp)
set(OSG_APP_HEADERS include/${PROJECT_NAME}/osg_app.h)

## Create MOC wrappers for headers
qt4_wrap_cpp(OSG_APP_HEADERS_MOC ${OSG_APP_HEADERS})

rosbuild_add_library(${PROJECT_NAME}	${OSG_APP_HEADERS}
										${OSG_APP_SOURCES}
										${OSG_APP_HEADERS_MOC})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(osg_app 	${OSG_APP_SOURCES})
target_link_libraries(osg_app 		${PROJECT_NAME}
																	${QT_LIBRARIES}
						  										${OPENSCENEGRAPH_LIBRARIES}
						  										osgQt)
