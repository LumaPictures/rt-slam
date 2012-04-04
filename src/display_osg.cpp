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


	const double ViewerOsg::DEFAULT_ELLIPSES_SCALE = 3.0;

	ViewerOsg::ViewerOsg(double _ellipsesScale):
			ellipsesScale(_ellipsesScale)
	{
		// load the scene.
		_root = new osg::Group;
		_root->setDataVariance(osg::Object::DYNAMIC);
//		//osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/Developer/Projects/rtslam/cow.osg");
		osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/DevProj/AR/rt-slam//cow.osg");
//		if (loadedModel) _root->addChild(loadedModel);
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
		group = new osg::Group;
		group->setDataVariance(osg::Object::DYNAMIC);
		viewerOsg->root()->addChild(group);
	}

	LandmarkOsg::~LandmarkOsg()
	{
		viewerOsg->root()->removeChild(group);
	}

	unsigned int LandmarkOsg::numShapes()
	{
		return group->getNumChildren();
	}

	void LandmarkOsg::clearShapes()
	{
		group->removeChildren(0, group->getNumChildren());
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

//					// ellipsoid
//					gdhe::Ellipsoid *ell = new gdhe::Ellipsoid(12);
//					ell->setLabel("");
//					items_.push_back(ell);
//					viewerGdhe->client.addObject(ell, false);
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
//					(*it)->setPose(position(0), position(1), position(2), 0, 0, 0);
//					(*it)->setLabelColor(c.R,c.G,c.B);
//					(*it)->setLabel(jmath::toStr(id_));
//					(*it)->refresh();

//					// ellipsoid
//					ItemList::iterator it = items_.begin();
//					gdhe::Ellipsoid *ell = PTR_CAST<gdhe::Ellipsoid*>(*it);
//					ell->set(state_, cov_, viewerGdhe->ellipsesScale);
//					c = getColorRGB(ColorManager::getColorObject_prediction(phase_,events_)) ;
//					(*it)->setColor(c.R,c.G,c.B); //
//					(*it)->setLabelColor(c.R,c.G,c.B);
//					(*it)->setLabel(jmath::toStr(id_));
//					(*it)->refresh();
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

//					// ellipsoid
//					gdhe::Ellipsoid *ell = new gdhe::Ellipsoid(12);
//					ell->setLabel("");
//					items_.push_back(ell);
//					viewerGdhe->client.addObject(ell, false);
//
//					// segment
//					gdhe::Polyline *seg = new gdhe::Polyline();
//					items_.push_back(seg);
//					viewerGdhe->client.addObject(seg, false);
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

//					// ellipsoid
//					ItemList::iterator it = items_.begin();
//					gdhe::Ellipsoid *ell = PTR_CAST<gdhe::Ellipsoid*>(*it);
//					jblas::vec xNew; jblas::sym_mat pNew; slamLmk_->reparametrize(LandmarkEuclideanPoint::size(), xNew, pNew);
////std::cout << "x_ahp " << state_ << " P_ahp " << cov_ << " ; x_euc " << xNew << " P_euc " << pNew << std::endl;
//					ell->setCompressed(xNew, pNew, viewerGdhe->ellipsesScale);
////					ell->set(xNew, pNew, viewerGdhe->ellipsesScale);
//					c = getColorRGB(ColorManager::getColorObject_prediction(phase_,events_)) ;
//					(*it)->setColor(c.R,c.G,c.B); //
//					(*it)->setLabelColor(c.R,c.G,c.B);
//					(*it)->setLabel(jmath::toStr(id_));
//					(*it)->refresh();
//
//
//					// segment
//					++it;
//					gdhe::Polyline *seg = PTR_CAST<gdhe::Polyline*>(*it);
//					seg->clear();
//					double id_std = sqrt(cov_(6,6))*viewerGdhe->ellipsesScale;
//					jblas::vec3 position = lmkAHP::ahp2euc(state_);
//					jblas::vec7 state = state_;
//					state(6) = state_(6) - id_std; if (state(6) < 1e-4) state(6) = 1e-4;
//					jblas::vec3 positionExt = lmkAHP::ahp2euc(state);
//					seg->addPoint(positionExt(0)-position(0), positionExt(1)-position(1), positionExt(2)-position(2));
//					state(6) = state_(6) + id_std;
//					positionExt = lmkAHP::ahp2euc(state);
//					seg->addPoint(positionExt(0)-position(0), positionExt(1)-position(1), positionExt(2)-position(2));
//					(*it)->setColor(c.R,c.G,c.B);
//					(*it)->setPose(position(0), position(1), position(2), 0, 0, 0);
//					(*it)->refresh();
				}
				break;
         }
         case LandmarkAbstract::LINE_AHPL:
         {
//            // Build display objects if it is the first time they are displayed
//            #ifdef DISPLAY_SEGMENT_DEPTH
//               if (items_.size() != 5)
//            #else
//               if (items_.size() != 3)
//            #endif
//            {
//               // clear
//               items_.clear();
//
//               // ellipsoids
//               gdhe::Ellipsoid *ell = new gdhe::Ellipsoid(12);
//               ell->setLabel("");
//               items_.push_back(ell);
//               viewerGdhe->client.addObject(ell, false);
//               ell = new gdhe::Ellipsoid(12);
//               ell->setLabel("");
//               items_.push_back(ell);
//               viewerGdhe->client.addObject(ell, false);
//               // segments
//               gdhe::Polyline *seg = new gdhe::Polyline();
//               items_.push_back(seg);
//               viewerGdhe->client.addObject(seg, false);
//               #ifdef DISPLAY_SEGMENT_DEPTH
//                   seg = new gdhe::Polyline();
//                   items_.push_back(seg);
//                   viewerGdhe->client.addObject(seg, false);
//                   seg = new gdhe::Polyline();
//                   items_.push_back(seg);
//                   viewerGdhe->client.addObject(seg, false);
//               #endif
//            }
//            // Refresh the display objects every time
//            {
//               colorRGB c; c.set(255,255,255);
//
//               // ellipsoids
//               ItemList::iterator it = items_.begin();
//               jblas::vec xNew;  jblas::sym_mat pNew;
//               jblas::vec xNew1; jblas::sym_mat pNew1;
//               jblas::vec xNew2; jblas::sym_mat pNew2;
//               slamLmk_->reparametrize(LandmarkEuclideanPoint::size()*2, xNew, pNew);
//               //slamLmk_->reparametrize(LandmarkAnchoredHomogeneousPointsLine::reparamSize(), xNew, pNew);
//               xNew1 = subrange(xNew,0,3);
//               xNew2 = subrange(xNew,3,6);
//               pNew1 = subrange(pNew,0,3,0,3);
//               pNew2 = subrange(pNew,3,6,3,6);
//
//               gdhe::Ellipsoid *ell = PTR_CAST<gdhe::Ellipsoid*>(*it);
//               ell->setCompressed(xNew1, pNew1, viewerGdhe->ellipsesScale);
//               c = getColorRGB(ColorManager::getColorObject_prediction(phase_,events_)) ;
//               (*it)->setColor(c.R,c.G,c.B); //
//               (*it)->setLabelColor(c.R,c.G,c.B);
//               (*it)->setLabel(jmath::toStr(id_));
//               (*it)->refresh();
//               ++it;
//               ell = PTR_CAST<gdhe::Ellipsoid*>(*it);
//               ell->setCompressed(xNew2, pNew2, viewerGdhe->ellipsesScale);
//               c = getColorRGB(ColorManager::getColorObject_prediction(phase_,events_)) ;
//               (*it)->setColor(c.R,c.G,c.B); //
//               (*it)->setLabelColor(c.R,c.G,c.B);
//               (*it)->setLabel(jmath::toStr(id_));
//               (*it)->refresh();
//
//               // segments
//               gdhe::Polyline *seg;
//               #ifdef DISPLAY_SEGMENT_DEPTH
//                  ++it;
//                  seg = PTR_CAST<gdhe::Polyline*>(*it);
//                  seg->clear();
//                  double id_std = sqrt(cov_(6,6))*viewerGdhe->ellipsesScale;
//                  jblas::vec7 _state1 = subrange(state_,0,7);
//                  jblas::vec3 position = lmkAHP::ahp2euc(_state1);
//                  jblas::vec7 state = _state1;
//                  state(6) = _state1(6) - id_std; if (state(6) < 1e-4) state(6) = 1e-4;
//                  jblas::vec3 positionExt = lmkAHP::ahp2euc(state);
//                  seg->addPoint(positionExt(0)-position(0), positionExt(1)-position(1), positionExt(2)-position(2));
//                  state(6) = _state1(6) + id_std;
//                  positionExt = lmkAHP::ahp2euc(state);
//                  seg->addPoint(positionExt(0)-position(0), positionExt(1)-position(1), positionExt(2)-position(2));
//                  (*it)->setColor(c.R,c.G,c.B);
//                  (*it)->setPose(position(0), position(1), position(2), 0, 0, 0);
//                  (*it)->refresh();
//                  ++it;
//                  seg = PTR_CAST<gdhe::Polyline*>(*it);
//                  seg->clear();
//                  id_std = sqrt(cov_(6,6))*viewerGdhe->ellipsesScale;
//                  jblas::vec7 _state2 = subrange(state_,0,7);
//                  subrange(_state2,3,7) = subrange(state_,7,11);
//                  position = lmkAHP::ahp2euc(_state2);
//                  state = _state2;
//                  state(6) = _state2(6) - id_std; if (state(6) < 1e-4) state(6) = 1e-4;
//                  positionExt = lmkAHP::ahp2euc(state);
//                  seg->addPoint(positionExt(0)-position(0), positionExt(1)-position(1), positionExt(2)-position(2));
//                  state(6) = _state2(6) + id_std;
//                  positionExt = lmkAHP::ahp2euc(state);
//                  seg->addPoint(positionExt(0)-position(0), positionExt(1)-position(1), positionExt(2)-position(2));
//                  (*it)->setColor(c.R,c.G,c.B);
//                  (*it)->setPose(position(0), position(1), position(2), 0, 0, 0);
//                  (*it)->refresh();
//               #endif
//               // Linking segment
//				#ifdef HAVE_MODULE_DSEG
//					jblas::vec3 xMiddle = (xNew1 + xNew2)/2;
//					desc_img_seg_fv_ptr_t descriptorSpec = SPTR_CAST<DescriptorImageSegFirstView>(slamLmk_->descriptorPtr);
//					float left_extremity = 1.0;
//					float right_extremity = 1.0;
//					if(descriptorSpec != NULL) {
//						left_extremity = descriptorSpec->getLeftExtremity();
//						right_extremity = descriptorSpec->getRightExtremity();
//					}
//					xNew1 = left_extremity * (xNew1 - xMiddle) + xMiddle;
//					xNew2 = right_extremity * (xNew2 - xMiddle) + xMiddle;
//               ++it;
//               seg = PTR_CAST<gdhe::Polyline*>(*it);
//               seg->clear();
//               seg->addPoint(xNew1(0), xNew1(1), xNew1(2));
//               seg->addPoint(xNew2(0), xNew2(1), xNew2(2));
//               (*it)->setColor(c.R,c.G,c.B);
//               (*it)->setPose(0,0,0,0,0,0);
//               (*it)->refresh();
//				#endif
//            }
            break;
         }
			default:
				JFR_ERROR(RtslamException, RtslamException::UNKNOWN_FEATURE_TYPE, "Don't know how to display this type of landmark: " << type_);
		}
	}



	ObservationOsg::ObservationOsg(ViewerAbstract *_viewer, rtslam::ObservationAbstract *_slamLmk, SensorOsg *_dispMap):
		ObservationDisplay(_viewer, _slamLmk, _dispMap), viewerOsg(PTR_CAST<ViewerOsg*>(_viewer))
	{
	}

}}}

#endif //HAVE_OSG



