/**
 * \file display_osg.cpp
 * \date 25/03/2010
 * \author croussil
 * \ingroup rtslam
 */
#include "rtslam/display_osg.hpp"

#ifdef HAVE_OSG

#include <cmath>

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osg/Geode>
#include <osg/ShapeDrawable>

#include "rtslam/ahpTools.hpp"

namespace jafar {
namespace rtslam {
namespace display {

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
		osg::Vec3dArray* verts = new osg::Vec3dArray;

		if (vertPositions.size() % 3 != 0)
		{
			JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "vertPositions must have length divisible by 3");
		}
		for(std::size_t i = 0; i < vertPositions.size(); i += 3)
		{
			verts->push_back( osg::Vec3d( vertPositions[i],
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

		osg::ref_ptr<osg::Vec4dArray> colors = new osg::Vec4dArray;
		geometry->setColorArray( colors.get() );
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


	const double ViewerOsg::DEFAULT_ELLIPSES_SCALE = 3.0;

	ViewerOsg::ViewerOsg(double _ellipsesScale):
			ellipsesScale(_ellipsesScale)
	{
		// load the scene.
		_root = new osg::Group;
		_root->setDataVariance(osg::Object::DYNAMIC);
		//osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/Developer/Projects/rtslam/cow.osg");
		//osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/DevProj/AR/rt-slam//cow.osg");
		//if (loadedModel) _root->addChild(loadedModel);
		_viewer = new osgViewer::Viewer;
		_viewer->setCameraManipulator( new osgGA::TrackballManipulator );
		_viewer->setSceneData(_root);
		_viewer->realize();
	}

	void ViewerOsg::render()
	{
		BaseViewerClass::render();
		_viewer->frame();
	}

	osg::ref_ptr<osg::Group> ViewerOsg::root()
	{
		return _root;
	}

	WorldOsg::WorldOsg(ViewerAbstract *_viewer, rtslam::WorldAbstract *_slamWor, WorldDisplay *garbage):
		WorldDisplay(_viewer, _slamWor, garbage), OsgViewerHolder(_viewer)
	{
	}


	MapOsg::MapOsg(ViewerAbstract *_viewer, rtslam::MapAbstract *_slamMap, WorldOsg *_dispWorld):
		MapDisplay(_viewer, _slamMap, _dispWorld), OsgViewerHolder(_viewer)
	{
	}

	void MapOsg::bufferize()
	{
		poseQuat = ublas::subrange(slamMap_->state.x(), 0, 7);
	}

	RobotOsg::RobotOsg(ViewerAbstract *_viewer, rtslam::RobotAbstract *_slamRob, MapOsg *_dispMap):
		RobotDisplay(_viewer, _slamRob, _dispMap), OsgGroupHolder(_viewer)
	{}


	osg::ref_ptr<osg::PositionAttitudeTransform> RobotOsg::makeRobotGeo()
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> transform;
		transform = new osg::PositionAttitudeTransform;
		transform->setDataVariance(osg::Object::DYNAMIC);
		group->addChild(transform);
		osg::Geode* geode = new osg::Geode();
		geode->setDataVariance(osg::Object::STATIC);
		transform->addChild(geode);

		double vertsRaw[] = {
			-0.424232, -0.806469, -1.084160,
			-0.424232, -0.806469, 0.944964,
			-0.424232, 0.383133, -1.084160,
			-0.424232, 0.383133, 0.944964,
			0.338672, 0.383133, -1.084160,
			0.338672, 0.383133, 0.944964,
			0.338672, -0.806469, -1.084160,
			0.338672, -0.806469, 0.944964,
			-0.225474, -0.225474, 0.711647,
			-0.225474, -0.225474, 1.365039,
			-0.225474, 0.225474, 0.711647,
			-0.225474, 0.225474, 1.365039,
			0.225474, 0.225474, 0.711647,
			0.225474, 0.225474, 1.365039,
			0.225474, -0.225474, 0.711647,
			0.225474, -0.225474, 1.365039,
			0.426727, -0.426727, 1.728939,
			-0.426727, -0.426727, 1.728938,
			0.426727, 0.426727, 1.728939,
			-0.426727, 0.426727, 1.728938,
			0.195310, 0.282111, -1.237820,
			0.195310, 0.015068, -0.510367,
			0.195310, 0.609222, -1.117740,
			0.195310, 0.342179, -0.390287,
			0.543765, 0.609222, -1.117740,
			0.543765, 0.342179, -0.390287,
			0.543765, 0.282111, -1.237820,
			0.543765, 0.015068, -0.510367,
			0.598679, 0.264946, -1.350416,
			0.140395, 0.264946, -1.350416,
			0.140395, 0.695159, -1.192488,
			0.598679, 0.695159, -1.192488};

		unsigned int facesRaw[] = {
			4, 0, 1, 3, 2,
			4, 2, 3, 5, 4,
			4, 4, 5, 7, 6,
			4, 6, 7, 1, 0,
			4, 1, 7, 5, 3,
			4, 6, 0, 2, 4,
			4, 8, 9, 11, 10,
			4, 10, 11, 13, 12,
			4, 12, 13, 15, 14,
			4, 14, 15, 9, 8,
			4, 9, 15, 16, 17,
			4, 15, 13, 18, 16,
			4, 13, 11, 19, 18,
			4, 11, 9, 17, 19,
			4, 20, 21, 23, 22,
			4, 22, 23, 25, 24,
			4, 24, 25, 27, 26,
			4, 26, 27, 21, 20,
			4, 21, 27, 25, 23,
			4, 28, 29, 30, 31,
			4, 26, 20, 29, 28,
			4, 20, 22, 30, 29,
			4, 22, 24, 31, 30,
			4, 24, 26, 28, 31,
			4, 18, 19, 17, 16};

	std::vector<double> verts(begin(vertsRaw), end(vertsRaw));
	std::vector<unsigned int> faces(begin(facesRaw), end(facesRaw));

	osg::ref_ptr<osg::Geometry> geo = makeGeoFromVertFaceLists(verts, faces);
	geode->addDrawable(geo);
	return transform;
	}

	void RobotOsg::bufferize()
	{
		poseQuat = slamRob_->pose.x();
		poseQuatUncert = slamRob_->pose.P();
	}

	void RobotOsg::render()
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> geo;

		// Build display objects if it is the first time they are displayed
		if (numShapes() != 1)
		{
			clearShapes();
			geo = makeRobotGeo();
		}
		else
		{
			geo = group->getChild(0)->asTransform()->asPositionAttitudeTransform();
		}

		// Refresh the display objects every time
		{
//			// convert pose from quat to euler degrees
//			jblas::vec poseEuler(6);
//			jblas::vec angleEuler(3);
//			jblas::sym_mat uncertEuler(3);
//			ublas::subrange(poseEuler,0,3) = ublas::subrange(poseQuat,0,3);
//			quaternion::q2e(ublas::subrange(poseQuat,3,7), ublas::project(poseQuatUncert,ublas::range(3,7),ublas::range(3,7)), angleEuler, uncertEuler);
//			ublas::subrange(poseEuler,3,6) = angleEuler;
//			//ublas::subrange(poseEuler,3,6) = quaternion::q2e(ublas::subrange(poseQuat,3,7));
//			for(int i = 3; i < 6; ++i) poseEuler(i) = jmath::radToDeg(poseEuler(i));
//			std::swap(poseEuler(3), poseEuler(5)); // FIXME-EULER-CONVENTION
//			geo->setPosition(osg::Vec3d(poseEuler[0],
//					poseEuler[1],
//					poseEuler[2]));
//			geo->setAttitude(osg::Vec3d(poseEuler[3],
//					poseEuler[4],
//					poseEuler[5]));

			geo->setPosition(osg::Vec3d(poseQuat[0],
					poseQuat[1],
					poseQuat[2]
					));
			geo->setAttitude(osg::Quat(poseQuat[3],
					poseQuat[4],
					poseQuat[5],
					poseQuat[6]
					));
		}
	}


	SensorOsg::SensorOsg(ViewerAbstract *_viewer, rtslam::SensorExteroAbstract *_slamRob, RobotOsg *_dispMap):
		SensorDisplay(_viewer, _slamRob, _dispMap), OsgViewerHolder(_viewer)
	{
	}
//
	LandmarkOsg::LandmarkOsg(ViewerAbstract *_viewer, rtslam::LandmarkAbstract *_slamLmk, MapOsg *_dispMap):
		LandmarkDisplay(_viewer, _slamLmk, _dispMap), OsgGroupHolder(_viewer)
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

	void LandmarkOsg::setColor(osg::ref_ptr<osg::Group> transform, double r, double g, double b)
	{
		osg::ShapeDrawable* shape;
		shape = static_cast<osg::ShapeDrawable*>(transform->getChild(0)->asGeode()->getDrawable(0));
		shape->setColor(osg::Vec4d(r, g, b, 1.0));
	}

	void LandmarkOsg::setColor(osg::ref_ptr<osg::Group> transform, colorRGB color)
	{
		setColor(transform, color.R, color.G, color.B);
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> LandmarkOsg::makeSphere()
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> sphere;

		sphere = new osg::PositionAttitudeTransform;
		sphere->setDataVariance(osg::Object::DYNAMIC);
		group->addChild(sphere);
		osg::Geode* geode = new osg::Geode;
		geode->setDataVariance(osg::Object::DYNAMIC);
		sphere->addChild(geode);
		// FIXME: figure out proper way to set sphere scale
		osg::ShapeDrawable* sphereShape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0), viewerOsg->ellipsesScale/100.0));
		geode->addDrawable(sphereShape);
		return sphere;
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> LandmarkOsg::makeLine()
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> line;
		line = new osg::PositionAttitudeTransform;
		line->setDataVariance(osg::Object::DYNAMIC);
		group->addChild(line);
		osg::Geode* geode = new osg::Geode();
		geode->setDataVariance(osg::Object::DYNAMIC);
		line->addChild(geode);
		osg::Geometry* geometry = new osg::Geometry();
		geometry->setDataVariance(osg::Object::DYNAMIC);
		// Because we'll be updating positions...
		geometry->setUseVertexBufferObjects(true);
		geode->addDrawable(geometry);
		osg::Vec3dArray* verts = new osg::Vec3dArray;
		verts->push_back( osg::Vec3d( 0, 0, 0) );
		verts->push_back( osg::Vec3d( 0, 0, 0) );
		geometry->setVertexArray( verts );
		osg::DrawElementsUInt* linePrimative = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
		linePrimative->push_back(0);
		linePrimative->push_back(1);
		geometry->addPrimitiveSet(linePrimative);
		return line;
	}

	colorRGB LandmarkOsg::getColor()
	{
		colorRGB c;
		c.set(255,255,255);
		c = getColorRGB(ColorManager::getColorObject_prediction(phase_,events_));
		return c;
	}


	void LandmarkOsg::render()
	{
		colorRGB color = getColor();

		switch (lmkType_)
		{
			case LandmarkAbstract::PNT_EUC:
			{
				osg::ref_ptr<osg::PositionAttitudeTransform> sphere;

				// Build display objects if it is the first time they are displayed
				if (numShapes() != 1)
				{
					clearShapes();
					sphere = makeSphere();
				}
				else
				{
					sphere = group->getChild(0)->asTransform()->asPositionAttitudeTransform();
				}

				// Refresh the display objects every time
				{
					// sphere
					setColor(sphere, color);
					sphere->setPosition(osg::Vec3d(state_[0], state_[1], state_[2]));
				}
				break;
			}
			case LandmarkAbstract::PNT_AH:
			{
				osg::ref_ptr<osg::PositionAttitudeTransform> sphere;
				osg::ref_ptr<osg::PositionAttitudeTransform> line;

				// Build display objects if it is the first time they are displayed
				if (numShapes() != 2)
				{
					clearShapes();
					sphere = makeSphere();
					line = makeLine();
				}
				else
				{
					sphere = group->getChild(0)->asTransform()->asPositionAttitudeTransform();
					line = group->getChild(1)->asTransform()->asPositionAttitudeTransform();
				}

				// Refresh the display objects every time
				{
					// sphere
					setColor(sphere, color);
					jblas::vec xNew; jblas::sym_mat pNew; slamLmk_->reparametrize(LandmarkEuclideanPoint::size(), xNew, pNew);
					sphere->setPosition(osg::Vec3d(xNew[0], xNew[1], xNew[2]));

					// segment
					osg::Geometry* lineGeo = line->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
					osg::Vec3dArray* verts = dynamic_cast<osg::Vec3dArray*>(lineGeo->getVertexArray());

					// FIXME: figure out proper scaling
					//double id_std = sqrt(cov_(6,6))*viewerOsg->ellipsesScale;
					//double id_std = sqrt(cov_(6,6));
					double id_std = sqrt(cov_(6,6))/viewerOsg->ellipsesScale;
					jblas::vec3 position = lmkAHP::ahp2euc(state_);
					jblas::vec7 state = state_;
					state(6) = state_(6) - id_std; if (state(6) < 1e-4) state(6) = 1e-4;
					jblas::vec3 positionExt = lmkAHP::ahp2euc(state);
					jblas::vec3 p1 = positionExt - position;
					state(6) = state_(6) + id_std;
					positionExt = lmkAHP::ahp2euc(state);
					jblas::vec3 p2 = positionExt - position;

					(*verts)[0] = osg::Vec3d(p1[0], p1[1], p1[2]);
					(*verts)[1] = osg::Vec3d(p2[0], p2[1], p2[2]);
					line->setPosition(osg::Vec3d(position[0], position[1], position[2]));
				}
				break;
		}
		case LandmarkAbstract::LINE_AHPL:
		{
			break;
		}
			default:
				JFR_ERROR(RtslamException, RtslamException::UNKNOWN_FEATURE_TYPE, "Don't know how to display this type of landmark: " << type_);
		}
	}



	ObservationOsg::ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamLmk, SensorOsg *_dispMap):
		ObservationDisplay(_viewer, _slamLmk, _dispMap), OsgViewerHolder(_viewer)
	{
	}

}}}

#endif //HAVE_OSG



