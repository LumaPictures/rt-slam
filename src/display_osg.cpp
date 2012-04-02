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

	ViewerOsg::ViewerOsg(double _ellipsesScale):
			ellipsesScale(_ellipsesScale)
	{
		// load the scene.
		osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/Developer/Projects/rtslam/cow.osg");
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

	WorldOsg::WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage):
		WorldDisplay(_viewer, _slamWor, garbage), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
	}


	MapOsg::MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld):
		MapDisplay(_viewer, _slamMap, _dispWorld), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
	}
//
//	MapOsg::~MapOsg()
//	{
//	}
//
	void MapOsg::bufferize()
	{
		poseQuat = ublas::subrange(slamMap_->state.x(), 0, 7);
	}

//
//	void MapOsg::render()
//	{
//
//	}
//
//
//
//
	RobotOsg::RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap):
		RobotDisplay(_viewer, _slamRob, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
	}

//	RobotOsg::~RobotOsg()
//	{
//	}
//
	void RobotOsg::bufferize()
	{
		poseQuat = slamRob_->pose.x();
		poseQuatUncert = slamRob_->pose.P();
	}
//	void RobotOsg::render()
//	{
//	}


	SensorOsg::SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamRob, RobotOsg *_dispMap):
		SensorDisplay(_viewer, _slamRob, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
	}
//
	LandmarkOsg::LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap):
		LandmarkDisplay(_viewer, _slamLmk, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
		id_ = _slamLmk->id();
		lmkType_ = _slamLmk->type;
		state_.resize(_slamLmk->state.x().size());
		cov_.resize(_slamLmk->state.P().size1(),_slamLmk->state.P().size2());
	}

//	LandmarkOsg::~LandmarkOsg()
//	{
//	}
//
//
	void LandmarkOsg::bufferize()
	{
		uchar *events = (uchar*)&events_;
		memset(events, 0, sizeof(ObservationAbstract::Events));
		for(LandmarkAbstract::ObservationList::iterator obs = slamLmk_->observationList().begin(); obs != slamLmk_->observationList().end(); ++obs)
		{
			uchar *obsevents = (uchar*)&((*obs)->events);
			for(size_t i = 0; i < sizeof(ObservationAbstract::Events); i++) events[i] |= obsevents[i];
		}

		state_ = slamLmk_->state.x();
		cov_ = slamLmk_->state.P();
	}

	//	void LandmarkOsg::render()
//	{
//
//	}



	ObservationOsg::ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamLmk, SensorOsg *_dispMap):
		ObservationDisplay(_viewer, _slamLmk, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
	}

}}}

#endif //HAVE_OSG



