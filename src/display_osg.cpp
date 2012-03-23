/**
 * \file display_osg.cpp
 * \date 25/03/2010
 * \author croussil
 * \ingroup rtslam
 */

#include "rtslam/display_osg.hpp"

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>

#ifdef HAVE_OSG

namespace jafar {
namespace rtslam {
namespace display {

ViewerOsg::ViewerOsg()
{
	// load the scene.
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/DevProj/eclipse/OSG_viewerQT/cow.osg");
	_viewer = new osgViewer::Viewer;
	_viewer->setCameraManipulator( new osgGA::TrackballManipulator );
	_viewer->setSceneData(loadedModel);
	_viewer->realize();
}

void ViewerOsg::render()
{
	BaseViewerClass::render();
	_viewer->frame();
}

//	WorldOsg::WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage):
//		WorldDisplay(_viewer, _slamWor, garbage), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
//	{
//	}
//
//	MapOsg::MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld):
//		MapDisplay(_viewer, _slamMap, _dispWorld), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)), frame(NULL)
//	{ }
//
//	MapOsg::~MapOsg()
//	{
//	}
//
//	void MapOsg::bufferize()
//	{
//	}
//
//	void MapOsg::render()
//	{
//
//	}
//
//
//
//
//	RobotOsg::RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap):
//		RobotDisplay(_viewer, _slamRob, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)), robot(NULL), uncertEll(NULL), traj(NULL)
//	{
//	}
//
//	RobotOsg::~RobotOsg()
//	{
//	}
//
//	void RobotOsg::bufferize()
//	{
//	}
//
//	void RobotOsg::render()
//	{
//	}
//
//	SensorOsg::SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamRob, RobotOsg *_dispMap):
//		SensorDisplay(_viewer, _slamRob, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}
//
//	LandmarkOsg::LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap):
//		LandmarkDisplay(_viewer, _slamLmk, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
//	{
//	}
//
//	LandmarkOsg::~LandmarkOsg()
//	{
//	}
//
//
//	void LandmarkOsg::bufferize()
//	{
//	}
//
//	void LandmarkOsg::render()
//	{
//
//	}
//
//
//
//	ObservationOsg::ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamLmk, SensorOsg *_dispMap):
//		ObservationDisplay(_viewer, _slamLmk, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}

}}}

#endif //HAVE_OSG



