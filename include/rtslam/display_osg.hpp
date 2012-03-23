/**
 * \file display_osg.hpp
 * \date 22/03/2012
 * \author Paul Molodowitch
 * File defining a display architecture for OpenSceneGraph in a QT window
 * \ingroup rtslam
 */

#ifndef DISPLAY_OSG__HPP_
#define DISPLAY_OSG__HPP_

#include "jafarConfig.h"

#ifdef HAVE_OSG

#define HAVE_DISP_OSG

#include <osgViewer/Viewer>

#include "rtslam/display.hpp"


namespace jafar {
namespace rtslam {
namespace display {

	class WorldOsg;
	class MapOsg;
	class RobotOsg;
	class SensorOsg;
	class LandmarkOsg;
	class ObservationOsg;
	
	class ViewerOsg: public Viewer<WorldOsg,MapOsg,RobotOsg,SensorOsg,LandmarkOsg,ObservationOsg,boost::variant<int*> >
	{
		public:
			typedef Viewer<WorldOsg,MapOsg,RobotOsg,SensorOsg,LandmarkOsg,ObservationOsg,boost::variant<int*> > BaseViewerClass;

			// some configuration parameters
			ViewerOsg();
			void render();

		protected:
			osg::ref_ptr<osgViewer::Viewer> _viewer;
	};


	class WorldOsg : public WorldDisplay
	{
			ViewerOsg *viewerOsg;
		public:
			WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage):
				WorldDisplay(_viewer, _slamWor, garbage), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}			void bufferize() {}
			void render() {}
	};

	class MapOsg : public MapDisplay
	{
			ViewerOsg *viewerOsg;
		public:
			MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld):
				MapDisplay(_viewer, _slamMap, _dispWorld), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}
			void bufferize() {}
			void render() {}
	};

	class RobotOsg : public RobotDisplay
	{
			ViewerOsg *viewerOsg;
		public:
			RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap):
				RobotDisplay(_viewer, _slamRob, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}
			void bufferize() {}
			void render() {}
	};

	class SensorOsg : public SensorDisplay
	{
			ViewerOsg *viewerOsg;
		public:
			SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamSen, RobotOsg *_dispRob):
				SensorDisplay(_viewer, _slamSen, _dispRob), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}
			void bufferize() {}
			void render() {}
	};

	class LandmarkOsg : public LandmarkDisplay
	{
			ViewerOsg *viewerOsg;
		public:
			LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap):
				LandmarkDisplay(_viewer, _slamLmk, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}
			void bufferize() {}
			void render() {}
	};

	class ObservationOsg : public ObservationDisplay
	{
			ViewerOsg *viewerOsg;
		public:
			ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamObs, SensorOsg *_dispSen):
				ObservationDisplay(_viewer, _slamObs, _dispSen), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer)) {}
			void bufferize() {}
			void render() {}
	};


}}}

#endif //HAVE_OSG
#endif //DISPLAY_OSG__HPP_

