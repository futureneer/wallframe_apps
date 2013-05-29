#ifndef _vector_conversitons_h
#define _vector_conversitons_h

#include <tf_conversions/tf_eigen.h>
#include <osg/Vec3>
#include <osg/Vec2>
namespace modulair{

	static osg::Vec3 eigToOsg3(Eigen::Vector3d v){
		return osg::Vec3(v[0],v[1],v[2]);
	};
	static Eigen::Vector3d osgToEig3(osg::Vec3 v){
		return Eigen::Vector3d(v[0],v[1],v[2]);
	};

	static osg::Vec2 eigToOsg2(Eigen::Vector2d v){
		return osg::Vec2(v[0],v[1]);
	};
	static Eigen::Vector2d osgToEig2(osg::Vec2 v){
		return Eigen::Vector2d(v[0],v[1]);
	};

}
#endif