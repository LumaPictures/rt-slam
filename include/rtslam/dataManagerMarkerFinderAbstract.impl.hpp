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
		DataManagerMarkerFinderAbstract::
		DataManagerMarkerFinderAbstract(size_t maxMarkersPerId_)
			: maxMarkersPerId(maxMarkersPerId_)
		{}

		void DataManagerMarkerFinderAbstract::
		detectNew(raw_ptr_t data)
		{
			MarkerPtr markerP = detectMarker(data);
			if (markerP)
			{
				addMarker(markerP);
			}
		}

		MarkerPtr DataManagerMarkerFinderAbstract::
		markerPose(int id)
		{
			MarkerPtr markerAvg(new Marker);
			MarkerPtr sum = getMarkerSum(id);
			if (sum)
			{
				// We found the id in the map, so divide the sum
				// by the num of observations to get the average
				*markerAvg = *sum;
				markerAvg->pose /= markerObservations[id].size();

				// Now, renormalize the quaternion portion...
				// Note that this method of taking component-wise
				// averge of quaternions, then re-normalizing, is
				// an approximate "average", which should be good
				// enough if results are relatively clustered - it
				// will break down if results are far apart

				// For a fuller discussion of averaging quaternions, see:
				// http://www.soest.hawaii.edu/wessel/courses/gg711/pdf/Gramkow_2001_JMIV.pdf
				// and a summary of this, at:
				// http://objectmix.com/graphics/132645-averaging-quaternions-2.html#post460418
				Marker::range_type quat = markerAvg->quaternion();
				quat /= ublas::norm_2(quat);
			}
			return markerAvg;
		}

		MarkerPtr DataManagerMarkerFinderAbstract::
		getMarkerSum(int id, bool create)
		{
			IdMarkerMap::iterator markerSumIter = markerObsSums.find(id);
			if (markerSumIter == markerObsSums.end())
			{
				MarkerPtr sum;
				// We didn't find a sum for this id in the map...
				if (create)
				{
					sum.reset(new Marker);
					sum->id = id;
					markerObsSums.insert(IdMarkerMap::value_type(id, sum));
				}
				// If we don't want to create, and we didn't find,
				// do nothing - we will return a null ptr
				return sum;
			}
			else
			{
				// We found the sum in the map...
				return (*markerSumIter).second;
			}
		}

		void DataManagerMarkerFinderAbstract::
		addMarker(MarkerPtr newMarker)
		{
			MarkerList& markers = markerObservations[newMarker->id];
			MarkerPtr markerSum = getMarkerSum(newMarker->id, true);
			// Can't first do size()-maxMarkersPerId, then check if it's
			// > 0, because we're dealing with unsigned values
			if (markers.size() >= maxMarkersPerId)
			{
				size_t numToDelete = markers.size() - (maxMarkersPerId - 1);
				for(size_t i = 0; i < numToDelete; ++i)
				{
					// Remove elements from the front - these are the oldest entries
					markerSum->pose -= markers[0]->pose;
					markers.pop_front();
				}
			}
			markers.push_back(newMarker);
			markerSum->pose += newMarker->pose;
		}



	} // namespace ::rtslam
} // namespace jafar::

