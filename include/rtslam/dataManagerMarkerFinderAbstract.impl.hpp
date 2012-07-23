/**
 * \file dataManagerMarkerFinderAbstract.impl.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#include "rtslam/dataManagerMarkerFinderAbstract.hpp"

namespace jafar {
	namespace rtslam {
		template<class RawSpec, class SensorSpec>
		DataManagerMarkerFinderAbstract<RawSpec, SensorSpec>::
		DataManagerMarkerFinderAbstract(size_t maxMarkersPerId_)
			: maxMarkersPerId(maxMarkersPerId_)
		{}

		template<class RawSpec, class SensorSpec>
		void DataManagerMarkerFinderAbstract<RawSpec, SensorSpec>::
		detectNew(raw_ptr_t data)
		{
			MarkerPtr markerP = detectMarker(data);
			addMarker(markerP);
		}

		template<class RawSpec, class SensorSpec>
		void DataManagerMarkerFinderAbstract<RawSpec, SensorSpec>::
		addMarker(MarkerPtr newMarker)
		{
			MarkerList& markers = markerMap[newMarker->id];
			if (markers.size() >= maxMarkersPerId)
			{
				size_t numToDelete = markers.size() - (maxMarkersPerId - 1);
				// Remove elements from the front - these are the oldest entries
				markers.erase(markers.begin(), markers.begin() + numToDelete);
			}
			markers.push_back(newMarker);
		}



	} // namespace ::rtslam
} // namespace jafar::

