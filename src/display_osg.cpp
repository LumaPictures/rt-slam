/**
 * \file display_osg.cpp
 * \date 25/03/2010
 * \author croussil
 * \ingroup rtslam
 */
#include "rtslam/display_osg.hpp"

#ifdef HAVE_OSG

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
			osg::Vec4d color=osg::Vec4d(.5,.5,.5,1.0)  )
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

		osg::Vec4dArray* colors = new osg::Vec4dArray;
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
				osg::Vec4d(r, g, b, a));
	}

	osg::ref_ptr<osg::StateSet> lineSS;

	// Utility method to make a line segment
	osg::ref_ptr<osg::Geometry> makeLineGeo(osg::Vec3 p1 = osg::Vec3(0,0,0),
			osg::Vec3 p2 = osg::Vec3(0,0,0),
			osg::Vec4d color = osg::Vec4d(0,0,0,1.0),
			osg::Object::DataVariance variance = osg::Object::UNSPECIFIED)
	{
		if (not lineSS)
		{
			lineSS = new osg::StateSet;
			lineSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF );
		}

		osg::Geometry* geometry = new osg::Geometry();
		geometry->setDataVariance(variance);
		if(variance == osg::Object::DYNAMIC)
		{
			// Assume that if it's dynamic, we'll be changing endpoints, so
			// we'll want to use vertex buffer objects
			geometry->setUseDisplayList(false);
			// TODO: figure out how to "properly" use vertexBufferObjects
			// with dynamic geometry - try to look at
			// http://www.openscenegraph.org/projects/osg/attachment/wiki/Support/Tutorials/osgGPUMorph.3.zip
			// ...once the openscenegraph website is working again!
			//geometry->setUseVertexBufferObjects(true);
		}
		osg::Vec3Array* verts = new osg::Vec3Array;
		verts->push_back( p1 );
		verts->push_back( p2 );
		geometry->setVertexArray( verts );
		osg::DrawElementsUInt* linePrimative = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
		linePrimative->push_back(0);
		linePrimative->push_back(1);
		geometry->addPrimitiveSet(linePrimative);
		osg::Vec4dArray* colors = new osg::Vec4dArray;
		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
		colors->push_back(color);
		geometry->setStateSet(lineSS.get());
		return geometry;
	}

	//////////////////////////////////////////////////
	// ViewerOsg
	//////////////////////////////////////////////////

	const double ViewerOsg::DEFAULT_ELLIPSES_SCALE = 3.0;
	const double ViewerOsg::NEAR_CLIP = .01;
	const double ViewerOsg::FAR_CLIP = 100.0;

	ViewerOsg::ViewerOsg(double _ellipsesScale):
			ellipsesScale(_ellipsesScale)
	{
		// load the scene.
		root_ = new osg::Group;
		osg::StateSet* rootState = root_->getOrCreateStateSet();
		rootState->setMode(GL_LIGHTING, osg::StateAttribute::ON );
		rootState->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
		root_->setDataVariance(osg::Object::DYNAMIC);
		//osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/Developer/Projects/rtslam/cow.osg");
		//osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/DevProj/AR/rt-slam//cow.osg");
		//if (loadedModel) root_->addChild(loadedModel);
		viewer_ = new osgViewer::Viewer;
		setupView(viewer_);
		viewer_->realize();
	}

	void ViewerOsg::render()
	{
		BaseViewerClass::render();
		viewer_->frame();
	}

	osg::ref_ptr<osg::Group> ViewerOsg::root()
	{
		return root_;
	}

	void ViewerOsg::setupView(osg::ref_ptr<osgViewer::View> view)
	{
		viewer_->setSceneData(root_);

		// TODO: pressing space in the manipulators disables the hard near/far
		// clipping planes we set - figure out how to disable this (perhaps by
		// setting the home position?)
		// set up the camera manipulators.
		{
			osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

			keyswitchManipulator->addMatrixManipulator( '1', "Terrain", new osgGA::TerrainManipulator() );
			keyswitchManipulator->addMatrixManipulator( '2', "Trackball", new osgGA::TrackballManipulator() );
			keyswitchManipulator->addMatrixManipulator( '3', "Flight", new osgGA::FlightManipulator() );
			keyswitchManipulator->addMatrixManipulator( '4', "Drive", new osgGA::DriveManipulator() );

			viewer_->setCameraManipulator( keyswitchManipulator.get() );
		}

		// Set the near/far clipping planes
		// By default, osg tries to manage this - this is bad for us, as we
		// create a fair number of line segments that shoot off WAY into the
		// distance...
		osg::Camera* cam = viewer_->getCamera();
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
		// Draw the world axes
		osg::ref_ptr<osg::Geometry> xAxis = makeLineGeo(osg::Vec3(0,0,0),
				osg::Vec3(1,0,0), osg::Vec4d(.5,0,0,1), osg::Object::STATIC);
		osg::ref_ptr<osg::Geometry> yAxis = makeLineGeo(osg::Vec3(0,0,0),
				osg::Vec3(0,1,0), osg::Vec4d(0,.5,0,1), osg::Object::STATIC);
		osg::ref_ptr<osg::Geometry> zAxis = makeLineGeo(osg::Vec3(0,0,0),
				osg::Vec3(0,0,1), osg::Vec4d(0,0,.5,1), osg::Object::STATIC);
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
		RobotDisplay(_viewer, _slamRob, _dispMap), OsgGeoHolder(_viewer)
	{}

	void RobotOsg::bufferize()
	{
		poseQuat = slamRob_->pose.x();
		poseQuatUncert = slamRob_->pose.P();
	}

	bool RobotOsg::needCreateShapes()
	{
		return numShapes() != 1;
	}

	void RobotOsg::createShapes()
	{
		// TODO: make osg own "module", move models there
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
		std::string camFile = std::getenv("JAFAR_DIR");
		char lastChar = camFile[camFile.size()-1];
		if (lastChar != '/' and lastChar != '\\')
		{
			camFile += "/";
		}
		camFile += "modules/rtslam/data/models/camera.osg";
		osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(camFile);
		loadedModel->setDataVariance(osg::Object::STATIC);

		osg::ref_ptr<osg::PositionAttitudeTransform> transform;
		transform = new osg::PositionAttitudeTransform;
		transform->setDataVariance(osg::Object::DYNAMIC);
		group->addChild(transform);
		transform->addChild(loadedModel);
	}

	void RobotOsg::refreshShapes()
	{
		osg::PositionAttitudeTransform* geo = group->getChild(0)->asTransform()->asPositionAttitudeTransform();

		geo->setPosition(osg::Vec3(poseQuat[0],
				poseQuat[1],
				poseQuat[2]
				));
		geo->setAttitude(osg::Quat(
				poseQuat[4],
				poseQuat[5],
				poseQuat[6],
				poseQuat[3]
				));
	}


	SensorOsg::SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamRob, RobotOsg *_dispMap):
		SensorDisplay(_viewer, _slamRob, _dispMap), OsgViewerHolder(_viewer)
	{
	}
//
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
		shape->setColor(osg::Vec4d(r, g, b, a));
	}

	void LandmarkOsg::setSphereColor(osg::ref_ptr<osg::Group> transform, colorRGB color)
	{
		setSphereColor(transform, color.R/255.0, color.G/255.0, color.B/255.0);
	}

	void LandmarkOsg::setLineColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b, double a)
	{
		osg::Geometry* geo = transform->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
		osg::Vec4dArray* colors = dynamic_cast<osg::Vec4dArray*>(geo->getColorArray());
		if (colors->size() != 1)
		{
			colors->clear();
			colors->push_back(osg::Vec4d(r, g, b, a));
		}
		else
		{
			(*colors)[0] = osg::Vec4d(r, g, b, a);
		}

	}

	void LandmarkOsg::setLineColor(osg::ref_ptr<osg::Group> transform, colorRGB color)
	{
		setLineColor(transform, color.R/255.0, color.G/255.0, color.B/255.0);
	}


	osg::ref_ptr<osg::MatrixTransform> LandmarkOsg::makeSphere()
	{
		// TODO: make less dense spheres (maybe don't use ShapeDrawable at all?)
		// perhaps check out osgworks...
		osg::ShapeDrawable* sphereShape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0), 1.0));
		return makeMTransformForDrawable(sphereShape);
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> LandmarkOsg::makeLine()
	{
		colorRGB colorInt = getColor();
		osg::Vec4d color(colorInt.R/255.0, colorInt.G/255.0, colorInt.B/255.0, 1.0);
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

}}}

#endif //HAVE_OSG



