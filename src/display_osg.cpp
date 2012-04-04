/**
 * \file display_osg.cpp
 * \date 25/03/2010
 * \author croussil
 * \ingroup rtslam
 */

#include "rtslam/display_osg.hpp"
#include "rtslam/ahpTools.hpp"

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osg/Geode>
#include <osg/ShapeDrawable>

#ifdef HAVE_OSG

namespace jafar {
namespace rtslam {
namespace display {

	using namespace osg;

	const double ViewerOsg::DEFAULT_ELLIPSES_SCALE = 0.03;

	ViewerOsg::ViewerOsg(double _ellipsesScale):
			ellipsesScale(_ellipsesScale)
	{
		// load the scene.
		_root = new Group;
		_root->setDataVariance(Object::DYNAMIC);
//		//ref_ptr<Node> loadedModel = osgDB::readNodeFile("/Developer/Projects/rtslam/cow.osg");
//		ref_ptr<Node> loadedModel = osgDB::readNodeFile("/DevProj/AR/rt-slam//cow.osg");
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

	ref_ptr<Group> ViewerOsg::root()
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
		group = new Group;
		group->setDataVariance(Object::DYNAMIC);
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

	void LandmarkOsg::setColor(osg::ref_ptr<osg::Group> transform, float r, float g, float b)
	{
		ref_ptr<ShapeDrawable> shape;
		shape = static_cast<ShapeDrawable*>(transform->getChild(0)->asGeode()->getDrawable(0));
		shape->setColor(Vec4f(r, g, b, 1.0));
	}

	void LandmarkOsg::setColor(osg::ref_ptr<osg::Group> transform, colorRGB color)
	{
		setColor(transform, color.R, color.G, color.B);
	}

	void LandmarkOsg::setColor(osg::ref_ptr<osg::Group> transform)
	{
		setColor(transform, getColor());
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> LandmarkOsg::makeSphere()
	{
		ref_ptr<PositionAttitudeTransform> sphere;

		sphere = new PositionAttitudeTransform;
		sphere->setDataVariance(Object::DYNAMIC);
		group->addChild(sphere);
		ref_ptr<Geode> geode = new Geode;
		geode->setDataVariance(Object::DYNAMIC);
		sphere->addChild(geode);
		ShapeDrawable* sphereShape = new ShapeDrawable(new Sphere(Vec3(0,0,0), viewerOsg->ellipsesScale));
		geode->addDrawable(sphereShape);
		return sphere;
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
		switch (lmkType_)
		{
			case LandmarkAbstract::PNT_EUC:
			{
				ref_ptr<PositionAttitudeTransform> sphere;

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
					setColor(sphere);
					sphere->setPosition(Vec3d(state_[0], state_[1], state_[2]));
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
				ref_ptr<PositionAttitudeTransform> sphere;

				// Build display objects if it is the first time they are displayed
				if (numShapes() != 2)
				{
					clearShapes();
					sphere = makeSphere();

					// dummy segment;
					makeSphere();

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
				}

				// Refresh the display objects every time
				{
					colorRGB c; c.set(255,255,255);

					// sphere
					setColor(sphere);
					jblas::vec xNew; jblas::sym_mat pNew; slamLmk_->reparametrize(LandmarkEuclideanPoint::size(), xNew, pNew);
					sphere->setPosition(Vec3d(xNew[0], xNew[1], xNew[2]));

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



