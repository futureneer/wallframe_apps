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

## Had to provide xerces finder
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

## XercesC library
find_package(XercesC REQUIRED)
if (XERCESC_FOUND)
  include_directories (${XERCESC_INCLUDE_DIRS})
else (XERCESC_FOUND)
  message (SEND_ERROR "code in ${CMAKE_CURRENT_SOURCE_DIR} requires XercesC")
endif (XERCESC_FOUND)

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


## Qt MOC Generator (for Q_OBJECT behaviors)
set(_moc ${CMAKE_CURRENT_BINARY_DIR}/${_current_MOC})

## Set your source files here (anything thats has a Q_OBJECT in it)
set(PIC_FLYER_SOURCES src/modulair_app_pic_flyer/Model/CollectionStore.cpp src/modulair_app_pic_flyer/Model/CSVParser.cpp src/modulair_app_pic_flyer/Model/DisplayImage.cpp src/modulair_app_pic_flyer/Model/ImageStoreDir.cpp src/modulair_app_pic_flyer/Model/ManuscriptCollection.cpp src/modulair_app_pic_flyer/Model/ManuscriptCollectionFactory.cpp src/modulair_app_pic_flyer/Model/Manuscript.cpp src/modulair_app_pic_flyer/Model/ModelSettings.cpp src/modulair_app_pic_flyer/Model/Opening.cpp src/modulair_app_pic_flyer/Model/Page.cpp src/modulair_app_pic_flyer/Model/PictureCollection.cpp src/modulair_app_pic_flyer/Model/PictureCollectionFactory.cpp)
set(PIC_FLYER_HEADERS include/modulair_app_pic_flyer/Model/CollectionStore.h include/modulair_app_pic_flyer/Model/CSVParser.h include/modulair_app_pic_flyer/Model/DisplayImage.h include/modulair_app_pic_flyer/Model/ImageStoreDir.h include/modulair_app_pic_flyer/Model/IPictureCollection.h include/modulair_app_pic_flyer/Model/ManuscriptCollectionFactory.h include/modulair_app_pic_flyer/Model/ManuscriptCollection.h include/modulair_app_pic_flyer/Model/Manuscript.h include/modulair_app_pic_flyer/Model/ModelSettings.h include/modulair_app_pic_flyer/Model/Opening.h include/modulair_app_pic_flyer/Model/Page.h include/modulair_app_pic_flyer/Model/PictureCollectionFactory.h include/modulair_app_pic_flyer/Model/PictureCollection.h)

## Create MOC wrappers for headers
qt4_wrap_cpp(PIC_FLYER_HEADERS_MOC ${PIC_FLYER_HEADERS})

rosbuild_add_library(${PROJECT_NAME}	${PIC_FLYER_HEADERS} ${PIC_FLYER_SOURCES} ${PIC_FLYER_HEADERS_MOC})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(pic_flyer_app 	${PIC_FLYER_SOURCES})
target_link_libraries(pic_flyer_app 	${PROJECT_NAME} ${QT_LIBRARIES}	${OPENSCENEGRAPH_LIBRARIES} osgQt ${XERCESC_LIBRARIES})
