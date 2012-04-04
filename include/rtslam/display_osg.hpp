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
#include <osg/PositionAttitudeTransform>

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

		public:
			double ellipsesScale;
			static const double DEFAULT_ELLIPSES_SCALE;

		protected:
			osg::ref_ptr<osgViewer::Viewer> _viewer;
		private:
			osg::ref_ptr<osg::Group> _root;

		public:
			// some configuration parameters
			ViewerOsg(double _ellipsesScale = DEFAULT_ELLIPSES_SCALE);
			void render();
			osg::ref_ptr<osg::Group> root();
	};

	class WorldOsg : public WorldDisplay
	{
		protected:
			ViewerOsg *viewerOsg;
		public:
			WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage);
			void bufferize() {}
			void render() {}
	};



	class MapOsg : public MapDisplay
	{
		protected:
			// bufferized data
			jblas::vec poseQuat;
			// osg objects
			ViewerOsg *viewerOsg;
		public:
			MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld);
			void bufferize();
			void render() {}
	};

	class RobotOsg : public RobotDisplay
	{
		protected:
			// bufferized data
			jblas::vec poseQuat;
			jblas::sym_mat poseQuatUncert;
			// osg objects
			ViewerOsg *viewerOsg;
		public:
			RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap);
			void bufferize();
			void render() {}
	};

	class SensorOsg : public SensorDisplay
	{
		protected:
			ViewerOsg *viewerOsg;
		public:
			SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamSen, RobotOsg *_dispRob);
			void bufferize() {}
			void render() {}
	};

	class LandmarkOsg : public LandmarkDisplay
	{
		protected:
			// buffered data
			ObservationAbstract::Events events_;
			jblas::vec state_;
			jblas::sym_mat cov_;
			unsigned int id_;
			LandmarkAbstract::type_enum lmkType_;
			// osg objects
			osg::ref_ptr<osg::Group> group;
			ViewerOsg *viewerOsg;
		public:
			LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap);
			~LandmarkOsg();
			void bufferize();
			void render();

		protected:
			// Some utility functions
			inline unsigned int numShapes();
			inline void clearShapes();
			inline colorRGB getColor();
			inline void setColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b);
			inline void setColor(osg::ref_ptr<osg::Group> transform, colorRGB rgb);
			osg::ref_ptr<osg::PositionAttitudeTransform> makeSphere();
			osg::ref_ptr<osg::PositionAttitudeTransform> makeLine();
	};

	class ObservationOsg : public ObservationDisplay
	{
		protected:
			ViewerOsg *viewerOsg;
		public:
			ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamObs, SensorOsg *_dispSen);
			void bufferize() {}
			void render() {}
	};


}}}

#endif //HAVE_OSG
#endif //DISPLAY_OSG__HPP_

