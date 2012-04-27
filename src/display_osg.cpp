/**
 * \file display_osg.cpp
 * \date 25/03/2010
 * \author Paul Molodowitch
 * \ingroup rtslam
 */
#include "rtslam/display_osg.hpp"

#if defined(HAVE_OSG) && defined(HAVE_QT4)

#include <cmath>
#include <cstdlib>

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osg/Geode>
#include <osg/ApplicationUsage>
#include <osg/ShapeDrawable>

#include "rtslam/ahpTools.hpp"
#include "jmath/angle.hpp"

namespace jafar {
namespace rtslam {
namespace display {

	//////////////////////////////////////////////////
	// Utility macros / functions
	//////////////////////////////////////////////////

	// Utility begin / end templates for arrays
	template <typename T, size_t N>
	T* begin(T(&arr)[N]) { return &arr[0]; }
	template <typename T, size_t N>
	T* end(T(&arr)[N]) { return &arr[0]+N; }

	// Utility method to make poly geo
	// Expects a flat list of vertIndices - [x,y,z,x,y,z,...]
	// and a flat list of faceIndices, of the form
	// [numVertsInFace1, indexOfFace1Vert1, indexOfFace1Vert2, indexOfFace1Vert3, ...,
	//  numVertsInFace2, indexOfFace2Vert1, indexOfFace2Vert2, indexOfFace2Vert3, ...
	// ]
	osg::ref_ptr<osg::Geometry> makeGeoFromVertFaceLists(std::vector<double> vertPositions,
			std::vector<unsigned int> faceIndices,
			osg::Vec4 color=osg::Vec4(.5,.5,.5,1.0)  )
	{
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
		geometry->setDataVariance(osg::Object::STATIC);
		osg::Vec3Array* verts = new osg::Vec3Array;

		if (vertPositions.size() % 3 != 0)
		{
			JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "vertPositions must have length divisible by 3");
		}
		for(std::size_t i = 0; i < vertPositions.size(); i += 3)
		{
			verts->push_back( osg::Vec3( vertPositions[i],
					vertPositions[i+1],
					vertPositions[i+2]) );
		}
		geometry->setVertexArray( verts );
		unsigned int vertsInFace;
		for(std::size_t i = 0; i < faceIndices.size(); i += vertsInFace + 1)
		{
			vertsInFace = faceIndices[i];
			if (vertsInFace < 3)
			{
				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "faces must have at least 3 verts");
			}
			// Double check that we don't read past the end of faceIndices
			if ( i + vertsInFace >= faceIndices.size() )
			{
				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "faceIndices list too short");
			}

			osg::DrawElementsUInt* poly = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
			for (unsigned int j = 1; j <= vertsInFace; ++j)
			{

				poly->push_back(faceIndices[i + j]);
			}

			geometry->addPrimitiveSet(poly);
		}

		osg::Vec4Array* colors = new osg::Vec4Array;
		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
		colors->push_back( color );

		return geometry;
	}

	osg::ref_ptr<osg::Geometry> makeGeoFromVertFaceLists(std::vector<double> vertPositions,
			std::vector<unsigned int> faceIndices,
			double r, double g, double b, double a = 1.0  )
	{
		return makeGeoFromVertFaceLists(vertPositions, faceIndices,
				osg::Vec4(r, g, b, a));
	}

	osg::ref_ptr<osg::Group> loadGeoFile(std::string filename)
	{
		// TODO: make osg own jafar "module", move models there
		// TODO: find better way to make relative path than using
		// env vars - either find path of executable, and make relative to that,
		// or attach file directly to executable
		//
		// some os-specific ways to get the current executable:
		//
		// Mac OS X: _NSGetExecutablePath() (man 3 dyld)
		// Linux: readlink /proc/self/exe
		// Solaris: getexecname()
		// FreeBSD: sysctl CTL_KERN KERN_PROC KERN_PROC_PATHNAME -1
		// BSD with procfs: readlink /proc/curproc/file
		// Windows: GetModuleFileName() with hModule = NULL
		//
		char* jafarDir = std::getenv("JAFAR_DIR");
		if (not jafarDir)
		{
			JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "JAFAR_DIR environment variable not set");
		}
		std::string path = jafarDir;
		char lastChar = path[path.size()-1];
		if (lastChar != '/' and lastChar != '\\')
		{
			path += "/";
		}
		path += "modules/rtslam/data/models/" + filename;
		return osgDB::readNodeFile(path)->asGroup();
	}

	osg::ref_ptr<osg::StateSet> lineSS;

	// Utility method to make a line segment
	osg::ref_ptr<osg::Geometry> makeLineGeo(osg::Vec3Array& verts_,
			const osg::Vec4 color = osg::Vec4(0,0,0,1.0),
			const osg::Object::DataVariance variance = osg::Object::UNSPECIFIED,
			bool allocateNewVertArray=true)
	{
		if (not lineSS)
		{
			lineSS = new osg::StateSet;
			lineSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF );
		}

		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
		geometry->setDataVariance(variance);
		if(variance == osg::Object::DYNAMIC)
		{
			// Assume that if it's dynamic, we'll be changing endpoints, so
			// we'll want to use vertex buffer objects
			geometry->setUseDisplayList(false);
			// TODO: figure out whether we should use VBOs or not...?
			//geometry->setUseVertexBufferObjects(true);
			// Unsure whether to use this, if we do use VBOs...?
//			// Set VBO to STREAMING since attributes are changing per frame
//			osg::VertexBufferObject* vbo = geometry->getOrCreateVertexBufferObject();
//			vbo->setUsage (GL_STREAM_DRAW);
		}
		osg::Vec3Array* vertsP;
		if (allocateNewVertArray)
		{
			vertsP = new osg::Vec3Array;
			*vertsP = verts_;
		}
		else vertsP = &verts_;
		geometry->setVertexArray( vertsP );
		osg::DrawElementsUInt* linePrimative = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_STRIP, 0);
		for (size_t i=0; i < vertsP->size(); ++i) linePrimative->push_back(i);
		geometry->addPrimitiveSet(linePrimative);
		osg::Vec4Array* colors = new osg::Vec4Array;
		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
		colors->push_back(color);
		geometry->setStateSet(lineSS.get());
		return geometry;
	}

	osg::ref_ptr<osg::Geometry> makeLineGeo(const osg::Vec3 p1 = osg::Vec3(0,0,0),
			const osg::Vec3 p2 = osg::Vec3(0,0,0),
			const osg::Vec4 color = osg::Vec4(0,0,0,1.0),
			const osg::Object::DataVariance variance = osg::Object::UNSPECIFIED)
	{
		osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
		verts->push_back(p1);
		verts->push_back(p2);
		return makeLineGeo(*verts, color, variance, false);
	}

	//////////////////////////////////////////////////
	// Manipulators
	//////////////////////////////////////////////////

	LookThroughManipulator::LookThroughManipulator(const osg::Quat& rotation)
	{
		_distance = 0;
		_rotation = rotation;
	}

	void LookThroughManipulator::setByMatrix(const osg::Matrixd& matrix)
	{}

	void LookThroughManipulator::setTransformation( const osg::Vec3d& eye, const osg::Quat& rotation )
	{}

	void LookThroughManipulator::setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up )
	{}

	osg::Matrixd LookThroughManipulator::getMatrix() const
	{
	    osg::Vec3d nodeCenter;
	    osg::Quat nodeRotation;
	    computeNodeCenterAndRotation(nodeCenter,nodeRotation);
	    return osg::Matrixd::rotate(_rotation)*osg::Matrixd::rotate(nodeRotation)*osg::Matrix::translate(nodeCenter);
	}


	osg::Matrixd LookThroughManipulator::getInverseMatrix() const
	{
	    osg::Vec3d nodeCenter;
	    osg::Quat nodeRotation;
	    computeNodeCenterAndRotation(nodeCenter,nodeRotation);
	    return osg::Matrixd::translate(-nodeCenter)*osg::Matrixd::rotate(nodeRotation.inverse())*osg::Matrixd::rotate(_rotation.inverse());
	}

	void LookThroughManipulator::home(double currentTime)
	{}

	void LookThroughManipulator::computePosition(const osg::Vec3d& eye,const osg::Vec3d& center,const osg::Vec3d& up)
	{}


	bool LookThroughManipulator::performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy )
	{
		return true;
	}


	bool LookThroughManipulator::performMovementMiddleMouseButton( const double eventTimeDelta, const double dx, const double dy )
	{
		return true;
	}


	bool LookThroughManipulator::performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy )
	{
		return true;
	}

	void LookThroughManipulator::rotateTrackball( const float px0, const float py0,
			const float px1, const float py1, const float scale )
	{}

	void LookThroughManipulator::setRotation( const osg::Quat& rotation )
	{}


	//////////////////////////////////////////////////
	// Callbacks
	//////////////////////////////////////////////////

	TrackNodeCullCallback::TrackNodeCullCallback(osg::ref_ptr<osg::Node> _trackNode):
						trackNode_(_trackNode)
	{}

	void TrackNodeCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		if (doTraverse(nv))
		{
			traverse(node, nv);
		}
	}

	bool TrackNodeCullCallback::doTraverse(osg::NodeVisitor* nv)
	{
		osgUtil::CullVisitor* cullVisitor = dynamic_cast<osgUtil::CullVisitor*>(nv);
		if (cullVisitor == NULL) return true;
		osgViewer::View* view = dynamic_cast<osgViewer::View*>(cullVisitor->getCurrentCamera()->getView());
		if (view == NULL) return true;
		osgGA::CameraManipulator* baseManip = view->getCameraManipulator();
		LookThroughManipulator* trackManip = dynamic_cast<LookThroughManipulator*>(baseManip);
		if (trackManip == NULL)
		{
			// Check if we have a KeySwitchMatrixManipulator which is currently set to use a NodeTrackerManipulator
			osgGA::KeySwitchMatrixManipulator* switchManip = dynamic_cast<osgGA::KeySwitchMatrixManipulator*>(baseManip);
			if (switchManip == NULL) return true;
			trackManip = dynamic_cast<LookThroughManipulator*>(switchManip->getCurrentMatrixManipulator());
			if (trackManip == NULL) return true;
		}
		return (trackManip->getTrackNode() != trackNode_.get());
	}


	//////////////////////////////////////////////////
	// ViewerOsg
	//////////////////////////////////////////////////

	const double ViewerOsg::DEFAULT_ELLIPSES_SCALE = 3.0;
	const double ViewerOsg::NEAR_CLIP = .01;
	const double ViewerOsg::FAR_CLIP = 100.0;
#if COMPOSITE_VIEW
	ViewerOsg::ViewerOsg(int _numViews, std::string _modelFile,
			double _ellipsesScale):
			numViews_(_numViews),
#else
	ViewerOsg::ViewerOsg(std::string _modelFile, double _ellipsesScale):
#endif
			ellipsesScale(_ellipsesScale),
			camTrackRobotId(1),
			initialized_(false),
			modelFile_(_modelFile)
	{
		// load the scene.
		root_ = new osg::Group;
		root_->setName("root");
		osg::StateSet* rootState = root_->getOrCreateStateSet();
		rootState->setMode(GL_LIGHTING, osg::StateAttribute::ON );
		rootState->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
		root_->setDataVariance(osg::Object::DYNAMIC);
		if (not modelFile_.empty())
		{
			osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(modelFile_);
			if (not loadedModel.valid())
			{
				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "number of osg views must be between 1 and 4, inclusive");
			}
			root_->addChild(loadedModel);
		}

		camTrackNode = new osg::Group;
		camTrackNode->setName("camTrackNode");
	}

	void ViewerOsg::render()
	{
		if (not initialized_) initializeWindow();
		BaseViewerClass::render();
		viewer_->frame();
	}

	// The initialization of the window needs to be delayed until after the creation of the
	// QApplication, so we can't just create the ViewerQT when the ViewerOsg is created...
	void ViewerOsg::initializeWindow()
	{
		OsgWidget* qtWidget;
#if COMPOSITE_VIEW
		if (numViews_ == 1)
	    {
#endif // COMPOSITE_VIEW
			ViewerQT* singleView = new ViewerQT;
			qtWidget = singleView;
			viewer_ = singleView;
			views_.push_back(singleView);
#if COMPOSITE_VIEW
	    }
		else if (numViews_ <= 4 and numViews_ > 0)
		{
			CompositeViewerQT* compositeView = new CompositeViewerQT;
			qtWidget = compositeView;
			viewer_ = compositeView;

	        unsigned int winWidth = compositeView->width();
	        unsigned int winHeight = compositeView->height();

	        std::vector<ViewPosition> viewPositions;

	        switch(numViews_)
	        {
	        case 2:
//	        	viewPositions.push_back(TOP);
//	        	viewPositions.push_back(BOTTOM);
	        	viewPositions.push_back(LEFT);
	        	viewPositions.push_back(RIGHT);
	        	break;
	        case 3:
//	        	viewPositions.push_back(TOP);
//	        	viewPositions.push_back(BOTTOM_LEFT);
	        	viewPositions.push_back(LEFT);
	        	viewPositions.push_back(TOP_RIGHT);
	        	viewPositions.push_back(BOTTOM_RIGHT);
	        	break;
	        case 4:
	        	viewPositions.push_back(TOP_LEFT);
	        	viewPositions.push_back(TOP_RIGHT);
	        	viewPositions.push_back(BOTTOM_LEFT);
	        	viewPositions.push_back(BOTTOM_RIGHT);
	        	break;
	        }


	        for (std::vector<ViewPosition>::iterator viewPosIt = viewPositions.begin();
	        		viewPosIt != viewPositions.end();
	        		++viewPosIt)

	        {
	            osgViewer::View* newView = new osgViewer::View;
	            ViewPosition currentPos = *viewPosIt;
	            int viewLeft;
	            int viewBottom;
	            int viewWidth;
	            int viewHeight;
	            switch(currentPos)
	            {
	            case BOTTOM:
	            	viewLeft = 0;
	            	viewWidth = winWidth;
	            	viewBottom = 1;
	            	viewHeight = winHeight / 2;
	            	break;
	            case TOP:
	            	viewLeft = 0;
	            	viewWidth = winWidth;
	            	viewBottom = winHeight / 2;
	            	viewHeight = winHeight - winHeight / 2;  // include the extra pixel if height is an odd number
	            	break;
	            case LEFT:
	            	viewLeft = 0;
	            	viewWidth = winWidth / 2;
	            	viewBottom = 0;
	            	viewHeight = winHeight;
	            	break;
	            case RIGHT:
	            	viewLeft = winWidth / 2;
	            	viewWidth = winWidth - winWidth / 2;
	            	viewBottom = 0;
	            	viewHeight = winHeight;
	            	break;
	            case BOTTOM_LEFT:
	            	viewLeft = 0;
	            	viewWidth = winWidth / 2;
	            	viewBottom = 0;
	            	viewHeight = winHeight / 2;
	            	break;
	            case BOTTOM_RIGHT:
	            	viewLeft = winWidth / 2;
	            	viewWidth = winWidth - winWidth / 2;
	            	viewBottom = 0;
	            	viewHeight = winHeight / 2;
	            	break;
	            case TOP_LEFT:
	            	viewLeft = 0;
	            	viewWidth = winWidth / 2;
	            	viewBottom = winHeight / 2;
	            	viewHeight = winHeight - winHeight / 2;
	            	break;
	            case TOP_RIGHT:
	            	viewLeft = winWidth / 2;
	            	viewWidth = winWidth - winWidth / 2;
	            	viewBottom = winHeight / 2;
	            	viewHeight = winHeight - winHeight / 2;
	            	break;
	            default:
	            	JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "Unrecognized window placement value");
	            	// This is just here to stop compiler warnings;
	            	viewLeft = 0;
	            	viewWidth = winWidth;
	            	viewBottom = 0;
	            	viewHeight = winHeight;
	            	break;
	            }

	            osg::Camera* cam = newView->getCamera();

	            cam->setProjectionMatrixAsPerspective(30.0f,
	            		static_cast<double>(viewWidth)/static_cast<double>(viewHeight),
	            		NEAR_CLIP, FAR_CLIP);
	            cam->setViewport(viewLeft, viewBottom, viewWidth, viewHeight);

	            compositeView->addView(newView);
	            views_.push_back(newView);
	        }
		}
		else
		{
			JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "number of osg views must be between 1 and 4, inclusive");
		}
#endif // COMPOSITE_VIEW

		for(size_t i = 0; i < views_.size(); ++i)
		{
			osg::ref_ptr<osgViewer::View> view = views_[i];
			setupView(view, i);
		}

		initialized_ = true;
		qtWidget->show();
	}

	osg::ref_ptr<osg::Group> ViewerOsg::root()
	{
		return root_;
	}

	void ViewerOsg::setupView(osg::ref_ptr<osgViewer::View> view, size_t viewNum)
	{
		// TODO: change background color
		view->setSceneData(root_);

		// TOOD: disable "drifting" after drag and "fling" with mouse
		// TODO: pressing space in the manipulators disables the hard near/far
		// clipping planes we set - figure out how to disable this (perhaps by
		// setting the home position?)
		// set up the camera manipulators.
		{
			osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

			osg::ref_ptr<osgGA::TrackballManipulator> trackballFixedUpManip = new osgGA::TrackballManipulator();
			trackballFixedUpManip->setVerticalAxisFixed(true);

			osg::ref_ptr<LookThroughManipulator> lookThroughManip = new LookThroughManipulator(osg::Quat(
					-PI*0.5, osg::Vec3(0.0,1.0,0.0),
					 PI*0.5, osg::Vec3(1.0,0.0,0.0),
					    0.0, osg::Vec3(0.0,0.0,1.0)
					));
			nodeTrackManips.push_back(lookThroughManip);
			lookThroughManip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);

			osg::ref_ptr<osgGA::NodeTrackerManipulator> followManip = new osgGA::NodeTrackerManipulator();
			nodeTrackManips.push_back(followManip);
			followManip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
//			followManip->setHomePosition(osg::Vec3(-.00001, 0, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,1));
			followManip->setHomePosition(osg::Vec3(-.15, 0, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,1));

			keyswitchManipulator->addMatrixManipulator( '1', "Trackball (Fixed Up)", trackballFixedUpManip );
			keyswitchManipulator->addMatrixManipulator( '2', "Trackball (Free)", new osgGA::TrackballManipulator() );
			keyswitchManipulator->addMatrixManipulator( '3', "Terrain", new osgGA::TerrainManipulator() );
			keyswitchManipulator->addMatrixManipulator( '4', "Look Through Camera", lookThroughManip );
			keyswitchManipulator->addMatrixManipulator( '5', "Follow Camera", followManip );
			keyswitchManipulator->setHomePosition(osg::Vec3(-2.0, 1.0, 1.0), osg::Vec3(0,0,0), osg::Vec3(0,0,1));

			view->setCameraManipulator( keyswitchManipulator.get() );

			if (viewNum == 1)
			{
				keyswitchManipulator->selectMatrixManipulator(3);
			}
			else if (viewNum == 2)
			{
				keyswitchManipulator->selectMatrixManipulator(4);
			}

		}

		// Set the near/far clipping planes
		// By default, osg tries to manage this - this is bad for us, as we
		// create a fair number of line segments that shoot off WAY into the
		// distance...
		osg::Camera* cam = view->getCamera();
		OsgWidget* osgWidget = dynamic_cast<OsgWidget*>(viewer_.get());
		cam->setGraphicsContext(osgWidget->getGraphicsWindow());
		cam->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
		double fovy, aspectRatio, zNear, zFar;
		cam->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
		cam->setProjectionMatrixAsPerspective(fovy, aspectRatio, NEAR_CLIP, FAR_CLIP);


		// add the state manipulator
		view->addEventHandler(new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()));

		// This causes a seg fault - probably problems with QT?
		// For possible solutions, see:
		//   http://forum.openscenegraph.org/viewtopic.php?t=9342
		//   http://labs.qt.nokia.com/2011/06/03/threaded-opengl-in-4-8/
		//   http://labs.qt.nokia.com/2010/06/17/youre-doing-it-wrong/
//        // add the thread model handler
//        view->addEventHandler(new osgViewer::ThreadingHandler);

		// Toggling OUT of fullscreen (on linux) has some issues - disable for now
		// add the window size toggle handler
		//view->addEventHandler(new osgViewer::WindowSizeHandler);

		// add the stats handler
		view->addEventHandler(new osgViewer::StatsHandler);

		// add the help handler
		view->addEventHandler(new osgViewer::HelpHandler(osg::ApplicationUsage::instance()));

		// add the record camera path handler
		view->addEventHandler(new osgViewer::RecordCameraPathHandler);

		// add the LOD Scale handler
		view->addEventHandler(new osgViewer::LODScaleHandler);

		// add the screen capture handler
		view->addEventHandler(new osgViewer::ScreenCaptureHandler);

	}

	//////////////////////////////////////////////////
	// OsgGeoHolder
	//////////////////////////////////////////////////

	OsgGeoHolder::OsgGeoHolder(ViewerAbstract *_viewer):
		OsgViewerHolder(_viewer)
	{
		group = new osg::Group;
		group->setName("OsgGeoHolderGroup");
		group->setDataVariance(osg::Object::DYNAMIC);
		viewerOsg->root()->addChild(group);
	}

	OsgGeoHolder::~OsgGeoHolder()
	{
		viewerOsg->root()->removeChild(group);
	}

	void OsgGeoHolder::render()
	{
		// Build display objects if it is the first time they are displayed
		if (needCreateShapes())
		{
			clearShapes();
			createShapes();
		}

		// Refresh the display objects every time
		refreshShapes();
	}

	// Some utility funcs
	unsigned int OsgGeoHolder::numShapes()
	{
		return group->getNumChildren();
	}

	void OsgGeoHolder::clearShapes()
	{
		group->removeChildren(0, group->getNumChildren());
	}

	template<class TransformType>
	osg::ref_ptr<TransformType> OsgGeoHolder::makeTransformForDrawable(osg::ref_ptr<osg::Drawable> drawable,
			bool addToGroup)
	{
		osg::ref_ptr<TransformType> trans = new TransformType;
		osg::Geode* geode = new osg::Geode();

		// Make transfrom variance dynamic - if we're making a transform,
		// likely because we want to move it around
		trans->setDataVariance(osg::Object::DYNAMIC);
		// Set the geode's variance to match shape's
		osg::Object::DataVariance shapeVar = drawable->getDataVariance();
		geode->setDataVariance(shapeVar);

		trans->addChild(geode);
		geode->addDrawable(drawable);

		if (addToGroup) group->addChild(trans);

		return trans;
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> OsgGeoHolder::makePATransformForDrawable(osg::ref_ptr<osg::Drawable> geo,
			bool addToGroup)
	{
		return makeTransformForDrawable<osg::PositionAttitudeTransform>(geo, addToGroup);
	}

	osg::ref_ptr<osg::MatrixTransform> OsgGeoHolder::makeMTransformForDrawable(osg::ref_ptr<osg::Drawable> geo,
			bool addToGroup)
	{
		return makeTransformForDrawable<osg::MatrixTransform>(geo, addToGroup);
	}


	//////////////////////////////////////////////////
	// All DisplayDataAbstract subclasses
	//////////////////////////////////////////////////

	WorldOsg::WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage):
		WorldDisplay(_viewer, _slamWor, garbage), OsgViewerHolder(_viewer)
	{
	}


	MapOsg::MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld):
		MapDisplay(_viewer, _slamMap, _dispWorld), OsgGeoHolder(_viewer)
	{
	}

	void MapOsg::bufferize()
	{
		//poseQuat = ublas::subrange(slamMap_->state.x(), 0, 7);
	}

	bool MapOsg::needCreateShapes()
	{
		return numShapes() != 1;
	}

	void MapOsg::createShapes()
	{
		// TODO: make labels
		// Draw the world axes
		osg::ref_ptr<osg::Geometry> xAxis = makeLineGeo(osg::Vec3(0,0,0),
				osg::Vec3(1,0,0), osg::Vec4(.5,0,0,1), osg::Object::STATIC);
		osg::ref_ptr<osg::Geometry> yAxis = makeLineGeo(osg::Vec3(0,0,0),
				osg::Vec3(0,1,0), osg::Vec4(0,.5,0,1), osg::Object::STATIC);
		osg::ref_ptr<osg::Geometry> zAxis = makeLineGeo(osg::Vec3(0,0,0),
				osg::Vec3(0,0,1), osg::Vec4(0,0,.5,1), osg::Object::STATIC);
		osg::ref_ptr<osg::PositionAttitudeTransform> trans = makePATransformForDrawable(xAxis);
		osg::Geode& geode = *(trans->getChild(0)->asGeode());
		geode.addDrawable(yAxis);
		geode.addDrawable(zAxis);
	}

	void MapOsg::refreshShapes()
	{
		//poseQuat = ublas::subrange(slamMap_->state.x(), 0, 7);
	}

	RobotOsg::RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap):
		RobotDisplay(_viewer, _slamRob, _dispMap), OsgGeoHolder(_viewer),
		isInertial_(isCastableTo(slamRob_, RobotInertial*))
	{
	}

	void RobotOsg::bufferize()
	{
		poseQuat = slamRob_->pose.x();
		poseQuatUncert = slamRob_->pose.P();
	}

	bool RobotOsg::needCreateShapes()
	{
		return numShapes() != 2;
	}

	void RobotOsg::createShapes()
	{
		std::string geoFilename;
		// Add the geo for the robot
		if (isInertial_) geoFilename = "MTi.osg";
		else geoFilename = "axes_color.osg";
		osg::ref_ptr<osg::Group> loadedModel = loadGeoFile(geoFilename);
		loadedModel->setDataVariance(osg::Object::STATIC);

		osg::ref_ptr<osg::PositionAttitudeTransform> robotTransform;
		robotTransform = new osg::PositionAttitudeTransform;
		robotTransform->setName("RobotTransform");
		robotTransform->setDataVariance(osg::Object::DYNAMIC);
		group->addChild(robotTransform);
		robotTransform->addChild(loadedModel);
		if (not isInertial_) robotTransform->setScale(osg::Vec3d(.1, .1, .1));

		// Now add the path
		pathPts = new osg::Vec3Array;
		pathGeo = makeLineGeo(*pathPts, osg::Vec4f(0,1,0,1), osg::Object::DYNAMIC,
				false);
		pathIndices = dynamic_cast<osg::DrawElementsUInt*>(pathGeo->getPrimitiveSet(0));
		osg::ref_ptr<osg::PositionAttitudeTransform> pathGeoTrans = makePATransformForDrawable(pathGeo);

		// If the cam is tracking the robot, don't want to show the robot geo or the path...
		if(viewerOsg->camTrackRobotId == slamRob_->id())
		{
			TrackNodeCullCallback* cullCallback = new TrackNodeCullCallback(viewerOsg->camTrackNode);
			robotTransform->addChild(viewerOsg->camTrackNode);
			for(std::vector<osg::ref_ptr<osgGA::NodeTrackerManipulator> >::iterator it = viewerOsg->nodeTrackManips.begin();
					it != viewerOsg->nodeTrackManips.end();
					++it)
			{
				(*it)->setTrackNode(viewerOsg->camTrackNode);
			}
			robotTransform->addCullCallback(cullCallback);
			pathGeoTrans->addCullCallback(cullCallback);
		}
	}

	void RobotOsg::refreshShapes()
	{
		osg::PositionAttitudeTransform* robot = group->getChild(0)->asTransform()->asPositionAttitudeTransform();

		osg::Vec3 pos(poseQuat[0], poseQuat[1], poseQuat[2]);
		robot->setPosition(pos);
		robot->setAttitude(osg::Quat(
				poseQuat[4],
				poseQuat[5],
				poseQuat[6],
				poseQuat[3]
				));

		pathPts->push_back(pos);
		pathIndices->push_back(pathIndices->size());
		pathPts->dirty();
		pathIndices->dirty();
		pathGeo->dirtyBound();
	}


	SensorOsg::SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamSen, RobotOsg *_dispMap):
		SensorDisplay(_viewer, _slamSen, _dispMap), OsgGeoHolder(_viewer)
	{
	}

	void SensorOsg::bufferize()
	{
		poseQuat = slamSen_->globalPose();
	}

	bool SensorOsg::needCreateShapes()
	{
		return numShapes() != 1;
	}

	void SensorOsg::createShapes()
	{
		osg::ref_ptr<osg::Group> loadedModel = loadGeoFile("cameraFlea3.osg");
		loadedModel->setDataVariance(osg::Object::STATIC);

		osg::ref_ptr<osg::PositionAttitudeTransform> sensorTransform;
		sensorTransform = new osg::PositionAttitudeTransform;
		sensorTransform->setName("SensorTransform");
		sensorTransform->setDataVariance(osg::Object::DYNAMIC);
		group->addChild(sensorTransform);
		sensorTransform->addChild(loadedModel);

		// If the cam is tracking the robot, don't want to show the sensor geo
		if(viewerOsg->camTrackRobotId == slamSen_->robotPtr()->id())
		{
			TrackNodeCullCallback* cullCallback = new TrackNodeCullCallback(viewerOsg->camTrackNode);
			sensorTransform->addCullCallback(cullCallback);
		}
	}

	void SensorOsg::refreshShapes()
	{
		osg::PositionAttitudeTransform* cam = group->getChild(0)->asTransform()->asPositionAttitudeTransform();

		osg::Vec3 pos(poseQuat[0], poseQuat[1], poseQuat[2]);
		cam->setPosition(pos);
		cam->setAttitude(osg::Quat(
				poseQuat[4],
				poseQuat[5],
				poseQuat[6],
				poseQuat[3]
				));
	}

	LandmarkOsg::LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap):
		LandmarkDisplay(_viewer, _slamLmk, _dispMap), OsgGeoHolder(_viewer)
	{
		id_ = _slamLmk->id();
		lmkType_ = _slamLmk->type;
		state_.resize(_slamLmk->state.x().size());
		cov_.resize(_slamLmk->state.P().size1(),_slamLmk->state.P().size2());
	}

	void LandmarkOsg::bufferize()
	{
		events_.clear();
		for(LandmarkAbstract::ObservationList::iterator obs = slamLmk_->observationList().begin(); obs != slamLmk_->observationList().end(); ++obs)
		{
			events_ |= (*obs)->events;
		}
		state_ = slamLmk_->state.x();
		cov_ = slamLmk_->state.P();
	}

	void LandmarkOsg::setSphereColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b, double a)
	{
		osg::ShapeDrawable* shape;
		shape = dynamic_cast<osg::ShapeDrawable*>(transform->getChild(0)->asGeode()->getDrawable(0));
		shape->setColor(osg::Vec4(r, g, b, a));
	}

	void LandmarkOsg::setSphereColor(osg::ref_ptr<osg::Group> transform, colorRGB color)
	{
		setSphereColor(transform, color.R/255.0, color.G/255.0, color.B/255.0);
	}

	void LandmarkOsg::setLineColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b, double a)
	{
		osg::Geometry* geo = transform->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
		osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geo->getColorArray());
		if (colors->size() != 1)
		{
			colors->clear();
			colors->push_back(osg::Vec4(r, g, b, a));
		}
		else
		{
			(*colors)[0] = osg::Vec4(r, g, b, a);
		}

	}

	void LandmarkOsg::setLineColor(osg::ref_ptr<osg::Group> transform, colorRGB color)
	{
		setLineColor(transform, color.R/255.0, color.G/255.0, color.B/255.0);
	}


	osg::ref_ptr<osg::MatrixTransform> LandmarkOsg::makeSphere()
	{
		// TODO: make labels!
		// TODO: make less dense spheres (maybe don't use ShapeDrawable at all?)
		// perhaps check out osgworks...
		osg::ShapeDrawable* sphereShape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0), 1.0));
		return makeMTransformForDrawable(sphereShape);
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> LandmarkOsg::makeLine()
	{
		colorRGB colorInt = getColor();
		osg::Vec4 color(colorInt.R/255.0, colorInt.G/255.0, colorInt.B/255.0, 1.0);
		osg::Vec3 p1(0,0,0);
		osg::Vec3 p2(0,0,0);

		osg::ref_ptr<osg::Geometry> lineGeo = makeLineGeo(p1, p2, color, osg::Object::DYNAMIC);
		return makePATransformForDrawable(lineGeo);
	}

	void LandmarkOsg::getEllipsoidPose(jblas::vec3 _x, jblas::sym_mat33 _xCov,
			double _scale, osg::Matrix& matrix, bool compressed)
	{
		// Code adapted from modules/gdhe/src/client.cpp, Ellipsoid::set(...)
		namespace lapack = boost::numeric::bindings::lapack;
		jblas::vec lambda(3);
//		jblas::mat_column_major A(ublas::project(_xCov, ublas::range(0,3), ublas::range(0,3)));
		jblas::mat_column_major A(_xCov);
		// SYEV is buggy to get 3d orientation... using generic GESDD instead to do svd decomposition, without using the fact that xCov is symmetric
//		up_sym_adapt s_A(A);
//		int ierr = lapack::syev( 'V', s_A, lambda, lapack::optimal_workspace() );
		jblas::mat_column_major U(3, 3);
		jblas::mat_column_major VT(3, 3);
		int ierr = lapack::gesdd('A',A,lambda,U,VT);
		A = U;

		double dx = lambda(0) < 1e-6 ? 1e-3 : sqrt(lambda(0));
		double dy = lambda(1) < 1e-6 ? 1e-3 : sqrt(lambda(1));
		double dz = lambda(2) < 1e-6 ? 1e-3 : sqrt(lambda(2));
		dx *= _scale;
		dy *= _scale;
		dz *= _scale;
		if(compressed)
		{
			double &maxdim = (dx>dy ? (dx>dz ? dx : dz) : (dy>dz ? dy : dz));
			double &mindim = (dx<dy ? (dx<dz ? dx : dz) : (dy<dz ? dy : dz));
			maxdim = mindim;
		}

		matrix.makeIdentity();
		// A gives the rotation... multiply by d* for scale
		// osg and rtslam apparently use different row/column order
		matrix(0,0) = dx*A(0,0);
		matrix(1,0) = dy*A(0,1);
		matrix(2,0) = dz*A(0,2);
		matrix(0,1) = dx*A(1,0);
		matrix(1,1) = dy*A(1,1);
		matrix(2,1) = dz*A(1,2);
		matrix(0,2) = dx*A(2,0);
		matrix(1,2) = dy*A(2,1);
		matrix(2,2) = dz*A(2,2);

		// Then set the translation...
		matrix.setTrans(_x[0], _x[1], _x[2]);
	}

	void LandmarkOsg::setEllipsoidPose(osg::ref_ptr<osg::MatrixTransform> sphere,
				bool compressed)
		{
			jblas::vec xNew; jblas::sym_mat pNew; slamLmk_->reparametrize(LandmarkEuclideanPoint::size(), xNew, pNew);
			osg::Matrix mat;
			getEllipsoidPose(xNew, pNew, viewerOsg->ellipsesScale, mat, compressed);
			sphere->setMatrix(mat);
		}

	colorRGB LandmarkOsg::getColor()
	{
		colorRGB c;
		c.set(255,255,255);
		c = getColorRGB(ColorManager::getColorObject_prediction(phase_,events_));
		return c;
	}

	bool LandmarkOsg::needCreateShapes()
	{
		switch (lmkType_)
		{
			case LandmarkAbstract::PNT_EUC:
			{
				return (numShapes() != 1);
			}
			case LandmarkAbstract::PNT_AH:
			{
				return (numShapes() != 2);

			}
			case LandmarkAbstract::LINE_AHPL:
			{
				// TODO: implement this...
				return false;
			}
			default:
			{
				JFR_ERROR(RtslamException, RtslamException::UNKNOWN_FEATURE_TYPE, "Don't know how to display this type of landmark: " << type_);
				return false; // Just here to make the IDE error checker happy...
			}
		}
	}


	void LandmarkOsg::createShapes()
	{
		switch (lmkType_)
		{
			case LandmarkAbstract::PNT_EUC:
			{
				makeSphere();
				break;
			}
			case LandmarkAbstract::PNT_AH:
			{
				makeSphere();
				makeLine();
				break;
			}
			case LandmarkAbstract::LINE_AHPL:
			{
				// TODO: implement this...
				break;
			}
			default:
			{
				JFR_ERROR(RtslamException, RtslamException::UNKNOWN_FEATURE_TYPE, "Don't know how to display this type of landmark: " << type_);
				break; // Just here to make the IDE error checker happy...
			}
		}
	}

	void LandmarkOsg::refreshShapes()
	{
		colorRGB color = getColor();
		switch (lmkType_)
		{
			case LandmarkAbstract::PNT_EUC:
			{
				osg::ref_ptr<osg::MatrixTransform> sphere;
				sphere = group->getChild(0)->asTransform()->asMatrixTransform();
				setSphereColor(sphere, color);
				setEllipsoidPose(sphere, false);
				break;
			}
			case LandmarkAbstract::PNT_AH:
			{
				osg::ref_ptr<osg::MatrixTransform> sphere;
				osg::ref_ptr<osg::PositionAttitudeTransform> line;

				sphere = group->getChild(0)->asTransform()->asMatrixTransform();
				line = group->getChild(1)->asTransform()->asPositionAttitudeTransform();

				// sphere
				setSphereColor(sphere, color);
				setEllipsoidPose(sphere, true);

				// segment
				osg::Geometry* lineGeo = line->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
				osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(lineGeo->getVertexArray());

				double id_std = sqrt(cov_(6,6))*viewerOsg->ellipsesScale;
				jblas::vec3 position = lmkAHP::ahp2euc(state_);
				jblas::vec7 state = state_;
				state(6) = state_(6) - id_std; if (state(6) < 1e-4) state(6) = 1e-4;
				jblas::vec3 positionExt = lmkAHP::ahp2euc(state);
				jblas::vec3 p1 = positionExt - position;
				state(6) = state_(6) + id_std;
				positionExt = lmkAHP::ahp2euc(state);
				jblas::vec3 p2 = positionExt - position;

				(*verts)[0] = osg::Vec3(p1[0], p1[1], p1[2]);
				(*verts)[1] = osg::Vec3(p2[0], p2[1], p2[2]);
				line->setPosition(osg::Vec3(position[0], position[1], position[2]));
				setLineColor(line, color);
				verts->dirty();
				lineGeo->dirtyBound();
				break;
			}
			case LandmarkAbstract::LINE_AHPL:
			{
				// TODO: implement this...
				break;
			}
			default:
			{
				JFR_ERROR(RtslamException, RtslamException::UNKNOWN_FEATURE_TYPE, "Don't know how to display this type of landmark: " << type_);
				break; // Just here to make the IDE error checker happy...
			}
		}
	}

	ObservationOsg::ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamLmk, SensorOsg *_dispMap):
		ObservationDisplay(_viewer, _slamLmk, _dispMap), OsgViewerHolder(_viewer)
	{
	}

} //namespace display
} //namespace rtslam
} //namespace jafar

#endif //HAVE_OSG && HAVE_QT4



