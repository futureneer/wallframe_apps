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

## Find IRRLICHT Libraries #####

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ~/modulair_applications/modulair_core_applications/modulair_app_wallball)

find_package(Irrlicht REQUIRED)
if (IRRLICHT_FOUND)
  include_directories(${IRRLICHT_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${IRRLICHT_LIBRARIES})
else (IRRLICHT_FOUND)
  message (SEND_ERROR "code in ${CMAKE_CURRENT_SOURCE_DIR}/apps/WallBall requires Irrlicht")
endif (IRRLICHT_FOUND)

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


## Qt MOC Generator (for Q_OBJECT behaviors)
set(_moc ${CMAKE_CURRENT_BINARY_DIR}/${_current_MOC})

## Set your source files here (anything thats has a Q_OBJECT in it)
set(WALLBALL_APP_SOURCES 
#  src/cpp_example_app.cpp
  src/WallBall.cpp
  src/WallBallWidget.cpp
  src/Components/CController.cpp
  src/Components/CGraphicsObject.cpp
  src/Components/CKeyboardController.cpp
  src/Components/CKinectController.cpp
  src/Components/CPhysicsObject.cpp
  src/Components/CSpawnPoint.cpp
  src/GameObject/Component.cpp
  src/GameObject/GameObject.cpp
  src/GameObject/GameObjectManager.cpp
  src/GameObject/GOBall.cpp
  src/GameObject/GOScore.cpp
  src/GameObject/GOBlock.cpp
  src/GameObject/GOPortal.cpp
  src/GameObject/GOSpawnPoint.cpp
  src/GameObject/GOStar.cpp
  src/GameObject/LevelObjects.cpp
  src/Managers/GraphicsManager.cpp
  src/Managers/InputManager.cpp
  src/Managers/PhysicsManager.cpp
  )

set(WALLBALL_APP_HEADERS 
#  include/${PROJECT_NAME}/cpp_example_app.h
  include/WallBallWidget.h
  include/Components/CController.h
  include/Components/CGraphicsObject.h
  include/Components/CKeyboardController.h
  include/Components/CKinectController.h
  include/Components/CPhysicsObject.h
  include/Components/CSpawnPoint.h
  include/GameObject/Component.h
  include/GameObject/GameObject.h
  include/GameObject/GameObjectManager.h
  include/GameObject/GOBall.h
  include/GameObject/GOScore.h
  include/GameObject/GOBlock.h
  include/GameObject/GOPortal.h
  include/GameObject/GOSpawnPoint.h
  include/GameObject/GOStar.h
  include/GameObject/LevelObjects.h
  include/Managers/GraphicsManager.h
  include/Managers/InputManager.h
  include/Managers/PhysicsManager.h

  include/Box2D/Box2D.h
  include/Box2D/Rope/b2Rope.h
  include/Box2D/Dynamics/b2Body.h
  include/Box2D/Dynamics/Contacts/b2PolygonContact.h
  include/Box2D/Dynamics/Contacts/b2Contact.h
  include/Box2D/Dynamics/Contacts/b2CircleContact.h
  include/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.h
  include/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.h
  include/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.h
  include/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.h
  include/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h
  include/Box2D/Dynamics/Contacts/b2ContactSolver.h
  include/Box2D/Dynamics/Joints/b2MouseJoint.h
  include/Box2D/Dynamics/Joints/b2RevoluteJoint.h
  include/Box2D/Dynamics/Joints/b2DistanceJoint.h
  include/Box2D/Dynamics/Joints/b2PrismaticJoint.h
  include/Box2D/Dynamics/Joints/b2FrictionJoint.h
  include/Box2D/Dynamics/Joints/b2WeldJoint.h
  include/Box2D/Dynamics/Joints/b2Joint.h
  include/Box2D/Dynamics/Joints/b2GearJoint.h
  include/Box2D/Dynamics/Joints/b2WheelJoint.h
  include/Box2D/Dynamics/Joints/b2PulleyJoint.h
  include/Box2D/Dynamics/Joints/b2RopeJoint.h
  include/Box2D/Dynamics/b2World.h
  include/Box2D/Dynamics/b2Island.h
  include/Box2D/Dynamics/b2TimeStep.h
  include/Box2D/Dynamics/b2ContactManager.h
  include/Box2D/Dynamics/b2Fixture.h
  include/Box2D/Collision/b2TimeOfImpact.h
  include/Box2D/Collision/b2Collision.h
  include/Box2D/Collision/b2Distance.h
  include/Box2D/Collision/b2DynamicTree.h
  include/Box2D/Collision/Shapes/b2PolygonShape.h
  include/Box2D/Collision/Shapes/b2Shape.h
  include/Box2D/Collision/Shapes/b2CircleShape.h
  include/Box2D/Collision/Shapes/b2EdgeShape.h
  include/Box2D/Collision/Shapes/b2ChainShape.h
  include/Box2D/Collision/b2BroadPhase.h
  include/Box2D/Common/b2GrowableStack.h
  include/Box2D/Common/b2Timer.h
  include/Box2D/Common/b2BlockAllocator.h
  include/Box2D/Common/b2Draw.h
  include/Box2D/Common/b2StackAllocator.h
  include/Box2D/Common/b2Math.h
  include/Box2D/Common/b2Settings.h

  )

set(WALLBALL_APP_LIBRARIES 
  ~/modulair_applications/modulair_core_applications/modulair_app_wallball/useLibs/Box2D.a
  )


## Create MOC wrappers for headers
qt4_wrap_cpp(WALLBALL_APP_HEADERS_MOC ${WALLBALL_APP_HEADERS})

rosbuild_add_library(${PROJECT_NAME}	${WALLBALL_APP_HEADERS}
										${WALLBALL_APP_SOURCES}
										${WALLBALL_APP_HEADERS_MOC})

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/useLibs)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(wallball_app 	${WALLBALL_APP_SOURCES})
target_link_libraries(wallball_app 		${PROJECT_NAME}
											${QT_LIBRARIES}
                                            ${IRRLICHT_LIBRARIES}
                                            ${WALLBALL_APP_LIBRARIES})
