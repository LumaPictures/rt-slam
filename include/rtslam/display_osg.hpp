/**
 * \file display_osg.hpp
 * \date 22/03/2012
 * \author Paul Molodowitch
 * File defining a display architecture for OpenSceneGraph in a QT window
 * \ingroup rtslam
 */

#ifndef DISPLAY_OSG__HPP_
#define DISPLAY_OSG__HPP_

/*
 * COMPOSITE_VIEW: allow support for a CompositeViewer osg window
 */
#define COMPOSITE_VIEW 1

#include "jafarConfig.h"

#if defined(HAVE_OSG) && defined(HAVE_QT4)

#define HAVE_DISP_OSG

#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>

#include "rtslam/display.hpp"
#include "rtslam/osgWidget.hpp"


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
			enum ViewPosition { TOP, BOTTOM, LEFT, RIGHT,
				TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT };
			typedef Viewer<WorldOsg,MapOsg,RobotOsg,SensorOsg,LandmarkOsg,ObservationOsg,boost::variant<int*> > BaseViewerClass;

#if COMPOSITE_VIEW
		protected:
			int numViews_;
#endif

		public:
			double ellipsesScale;
			static const double DEFAULT_ELLIPSES_SCALE;
			static const double NEAR_CLIP;
			static const double FAR_CLIP;

		protected:
			osg::ref_ptr<osgViewer::ViewerBase> viewer_;
			std::vector<osg::ref_ptr<osgViewer::View> > views_;
			bool initialized_;
		private:
			osg::ref_ptr<osg::Group> root_;

		public:
			// some configuration parameters
#if COMPOSITE_VIEW
			ViewerOsg(int numViews, double _ellipsesScale = DEFAULT_ELLIPSES_SCALE);
#else
			ViewerOsg(double _ellipsesScale = DEFAULT_ELLIPSES_SCALE);
#endif
			void render();
			osg::ref_ptr<osg::Group> root();

		protected:
			void initializeWindow();
			void setupView(osg::ref_ptr<osgViewer::View> view);
	};

	class OsgViewerHolder
	{
		protected:
			ViewerOsg *viewerOsg;
		public:
			OsgViewerHolder(ViewerAbstract *_viewer):
				viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
			{}
	};

	class OsgGeoHolder : public OsgViewerHolder
	{
		protected:
			osg::ref_ptr<osg::Group> group;
		public:
			OsgGeoHolder(ViewerAbstract *_viewer);
			virtual ~OsgGeoHolder();
			void render();

		protected:
			// Override these in derived classes!
			virtual bool needCreateShapes() = 0;
			virtual void createShapes() = 0;
			virtual void refreshShapes() = 0;

			// Some utility funcs
			unsigned int numShapes();
			void clearShapes();

			template<class TransformType>
			osg::ref_ptr<TransformType> makeTransformForDrawable(osg::ref_ptr<osg::Drawable> geo, bool addToGroup=true);

			osg::ref_ptr<osg::PositionAttitudeTransform> makePATransformForDrawable(osg::ref_ptr<osg::Drawable> geo, bool addToGroup=true);
			osg::ref_ptr<osg::MatrixTransform> makeMTransformForDrawable(osg::ref_ptr<osg::Drawable> geo, bool addToGroup=true);
	};

	class WorldOsg : public WorldDisplay, public OsgViewerHolder
	{
		public:
			WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage);
			void bufferize() {}
			void render() {}
	};

	class MapOsg : public MapDisplay, public OsgGeoHolder
	{
		protected:
			// bufferized data
			//jblas::vec poseQuat;
		public:
			MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld);
			void bufferize();
		protected:
			virtual bool needCreateShapes();
			virtual void createShapes();
			virtual void refreshShapes();
	};

	class RobotOsg : public RobotDisplay, public OsgGeoHolder
	{
		protected:
			// bufferized data
			jblas::vec poseQuat;
			jblas::sym_mat poseQuatUncert;
			osg::ref_ptr<osg::Geometry> pathGeo;
			osg::ref_ptr<osg::Vec3Array> pathPts;
			osg::ref_ptr<osg::DrawElementsUInt> pathIndices;
		public:
			RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap);
			void bufferize();
		protected:
			virtual bool needCreateShapes();
			virtual void createShapes();
			virtual void refreshShapes();
	};

	class SensorOsg : public SensorDisplay, public OsgViewerHolder
	{
		public:
			SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamSen, RobotOsg *_dispRob);
			void bufferize() {}
			void render() {}
	};

	class LandmarkOsg : public LandmarkDisplay, public OsgGeoHolder
	{
		protected:
			// buffered data
			ObservationAbstract::Events events_;
			jblas::vec state_;
			jblas::sym_mat cov_;
			unsigned int id_;
			LandmarkAbstract::type_enum lmkType_;
		public:
			LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap);
			void bufferize();

		protected:
			virtual bool needCreateShapes();
			virtual void createShapes();
			virtual void refreshShapes();

			// Some utility functions
			inline colorRGB getColor();
			inline void setSphereColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b, double a = 1.0);
			inline void setSphereColor(osg::ref_ptr<osg::Group> transform, colorRGB rgb);
			inline void setLineColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b, double a = 1.0);
			inline void setLineColor(osg::ref_ptr<osg::Group> transform, colorRGB rgb);
			osg::ref_ptr<osg::MatrixTransform> makeSphere();
			osg::ref_ptr<osg::PositionAttitudeTransform> makeLine();
			void getEllipsoidPose(jblas::vec3 _x, jblas::sym_mat33 _xCov,
						double _scale, osg::Matrix& outMatrix,
						bool compressed=false);
			void setEllipsoidPose(osg::ref_ptr<osg::MatrixTransform> sphereTrans,
						bool compressed=false);
	};

	class ObservationOsg : public ObservationDisplay, public OsgViewerHolder
	{
		public:
			ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamObs, SensorOsg *_dispSen);
			void bufferize() {}
			void render() {}
	};


}}}

#endif //HAVE_OSG && HAVE_QT4
#endif //DISPLAY_OSG__HPP_

