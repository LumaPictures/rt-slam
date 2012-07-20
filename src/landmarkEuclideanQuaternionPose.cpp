/*
 * \file landmarkEuclideanQuaternionPose.cpp
 * \date 20/08/2012
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#include "rtslam/landmarkEuclideanQuaternionPose.hpp"

namespace jafar {
	namespace rtslam {

		/**
		 * Constructor from map
		 */
		LandmarkEuclideanQuaternionPose::LandmarkEuclideanQuaternionPose(const map_ptr_t & mapPtr) :
				LandmarkAbstract(mapPtr, size()) {
			geomType = POSE,
			type = POSE_EUC_QUAT;
			converged = true;
		}

		LandmarkEuclideanQuaternionPose::LandmarkEuclideanQuaternionPose(const simulation_t dummy, const map_ptr_t & mapPtr) :
				LandmarkAbstract(FOR_SIMULATION, mapPtr, size()) {
			geomType = POSE,
			type = POSE_EUC_QUAT;
			converged = true;
		}

		/**
		 * Constructor from a previous lmk
		 */
		LandmarkEuclideanQuaternionPose::LandmarkEuclideanQuaternionPose(const map_ptr_t & _mapPtr, const landmark_ptr_t prevlmk,jblas::ind_array & _icomp ) :
				LandmarkAbstract(_mapPtr,prevlmk,size(),_icomp) {
			geomType = POSE,
			type = POSE_EUC_QUAT;
			converged = true;
		}

	} // namespace rtslam
} // namespace jafar
