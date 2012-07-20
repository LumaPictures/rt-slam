/**
 * \file dataManagerMarkerFinder.impl.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#include "rtslam/dataManagerMarkerFinder.hpp"

namespace jafar {
	namespace rtslam {

		template<class RawSpec, class SensorSpec>
		void DataManagerMarkerFinder<RawSpec, SensorSpec>::
		detectNew(raw_ptr_t data)
		{
			MarkerPtr markerP = detectMarker(data);
			if (markerP && mapManagerPtr()->mapSpaceForInit())
			{
				// Create the lmk and associated obs object.
				observation_ptr_t obsPtr =
						mapManagerPtr()->createNewLandmark(shared_from_this());
				LandmarkAbstract& landmark = obsPtr->landmark();
				switch(landmark.type)
				{
				case(LandmarkAbstract::POSE_EUC_QUAT):
					landmark.state.x() = markerP->pose;
					break;
				default:
					JFR_ERROR(RtslamException, RtslamException::UNKNOWN_FEATURE_TYPE, "Don't know how to convert marker to this type of landmark: " << landmark.type);
				}
			}
		}


	} // namespace ::rtslam
} // namespace jafar::

