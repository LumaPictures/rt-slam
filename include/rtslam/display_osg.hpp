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

#include <osg/Version>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osgGA/NodeTrackerManipulator>

#include "rtslam/display.hpp"
#include "rtslam/osgWidget.hpp"

#if OSG_MIN_VERSION_REQUIRED(3, 0, 0)
	typedef osgGA::CameraManipulator CameraManipulator;
	#define OSG_FIXED_UP_MANIPULATOR_AVAILABLE 1
#else
	typedef osgGA::MatrixManipulator CameraManipulator;
	#define OSG_FIXED_UP_MANIPULATOR_AVAILABLE 0
#endif

namespace jafar {
namespace rtslam {
namespace display {

	// Could do
	// #define _USE_MATH_DEFINES
	// #include <cmath>
	// ...but that requires that nobody else have already included cmath,
	// without first having set _USE_MATH_DEFINES
	// Rather than relying on this, just define our own...
	const double PI = std::atan(1.0)*4;

	class WorldOsg;
	class MapOsg;
	class RobotOsg;
	class SensorOsg;
	class LandmarkOsg;
	class ObservationOsg;
	
	// Easiest way to get a 'locked' NodeTrackerManipulator is to inherit from
	// it, then disable the ability to orbit... not exactly the prettiest
	// way of doing it, though...
	class LookThroughManipulator : public osgGA::NodeTrackerManipulator
	{
		public:
			LookThroughManipulator(const osg::Quat& rotation);
			void setByMatrix(const osg::Matrixd& matrix);
			void setTransformation( const osg::Vec3d& eye, const osg::Quat& rotation );
			void setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up );
			osg::Matrixd getMatrix() const;
			osg::Matrixd getInverseMatrix() const;
			void home(double currentTime);
			void computePosition(const osg::Vec3d& eye,const osg::Vec3d& center,const osg::Vec3d& up);
			bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy );
			bool performMovementMiddleMouseButton( const double eventTimeDelta, const double dx, const double dy );
			bool performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy );
			void rotateTrackball( const float px0, const float py0,
					const float px1, const float py1, const float scale );
			void setRotation( const osg::Quat& rotation );
	};

    class TrackNodeCullCallback : public osg::NodeCallback
    {
    	protected:
    		osg::ref_ptr<osg::Node> trackNode_;

		public:
			TrackNodeCullCallback(osg::ref_ptr<osg::Node> _trackNode);
			virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
			bool doTraverse(osg::NodeVisitor* nv);
    };

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
			osg::ref_ptr<osg::Group> camTrackNode;
			std::vector<osg::ref_ptr<osgGA::NodeTrackerManipulator> > nodeTrackManips;
			size_t camTrackRobotId;
			static const double DEFAULT_ELLIPSES_SCALE;
			static const double NEAR_CLIP;
			static const double FAR_CLIP;

		protected:
			osg::ref_ptr<osgViewer::ViewerBase> viewer_;
			std::vector<osg::ref_ptr<osgViewer::View> > views_;
			bool initialized_;
			std::string modelFile_;
		private:
			osg::ref_ptr<osg::Group> root_;

		public:
			// some configuration parameters
#if COMPOSITE_VIEW
			ViewerOsg(int numViews,
					std::string _modelFile = "",
					double _ellipsesScale = DEFAULT_ELLIPSES_SCALE);
#else
			ViewerOsg(std::string _modelFile = "",
					double _ellipsesScale = DEFAULT_ELLIPSES_SCALE);
);
#endif
			void render();
			osg::ref_ptr<osg::Group> root();

		protected:
			void initializeWindow();
			void setupView(osg::ref_ptr<osgViewer::View> view, size_t viewNum);
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

