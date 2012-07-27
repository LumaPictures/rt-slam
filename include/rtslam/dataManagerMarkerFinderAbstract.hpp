/**
 * \file dataManagerMarkerFinderAbstract.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#ifndef DATAMANAGERMARKERFINDERABSTRACT_HPP_
#define DATAMANAGERMARKERFINDERABSTRACT_HPP_

#include <map>
#include <deque>

#include "boost/shared_ptr.hpp"

#include "rtslam/dataManagerAbstract.hpp"
#include "rtslam/observationAbstract.hpp"

namespace jafar {
	namespace rtslam {

		/**
		This class holds data representing a single marker

		@ingroup rtslam
		*/
		struct Marker
		{
			typedef ublas::vector_range<jblas::vec7> range_type;
			Marker()
			{
				pose.clear();
				pose[3] = 1;
			}

			int id;
			// Jafar standard translation/quat - ie tx,ty,tz,qw,qx,qy,qz
			jblas::vec7 pose;

			range_type
			translation()
			{
				return subrange(pose, 0,3);
			}

			range_type
			quaternion()
			{
				return subrange(pose, 3,7);
			}
		};

		typedef boost::shared_ptr<Marker> MarkerPtr;
		typedef std::deque<MarkerPtr> MarkerList;
		typedef std::map<int, MarkerList > IdMarkerListMap;
		typedef std::map<int, MarkerPtr> IdMarkerMap;

		class DataManagerMarkerFinderAbstract;
		typedef boost::shared_ptr<DataManagerMarkerFinderAbstract> data_man_markerfinder_ptr_t;

		/**
		This class implements marker / tag searches

		@ingroup rtslam
		*/
		class DataManagerMarkerFinderAbstract: public DataManagerAbstract {
			public: // public interface
				static const size_t DEFAULT_MAX_MARKERS = 10;
				DataManagerMarkerFinderAbstract(float markerSize_,
						size_t maxMarkersPerId_=DEFAULT_MAX_MARKERS);

				void processKnown(raw_ptr_t data, double date_limit = -1.)
				{}

				void detectNew(raw_ptr_t data);

				// TODO: eventually, support multiple-marker tracking, and
				// convert this over to return a Marker list
				virtual MarkerPtr detectMarker(raw_ptr_t data) = 0;

				MarkerPtr markerPose(int id);

			protected: // main data members
				float markerSize;
				IdMarkerListMap markerObservations;
				IdMarkerMap markerObsSums;
				size_t maxMarkersPerId;


			protected: // parameters

			public: // getters and setters

			protected: // helper functions
				void addMarker(MarkerPtr newMarker);
				MarkerPtr getMarkerSum(int id, bool create=false);

		};

		template<class RawSpec>
		class DataManagerMarkerFinder: public DataManagerMarkerFinderAbstract, public SpecificChildOf<SensorPinhole> {
			public:
				// Define the function linkToParentSensorPinhole.
				ENABLE_LINK_TO_SPECIFIC_PARENT(SensorExteroAbstract, SensorPinhole, SensorPinhole, DataManagerAbstract);

				// Define the functions sensorPinhole() and sensorPinholePtr().
				ENABLE_ACCESS_TO_SPECIFIC_PARENT(SensorPinhole, sensorPinhole);
			public:
				DataManagerMarkerFinder(float markerSize_)
					: DataManagerMarkerFinderAbstract(markerSize_)
				{}
		};
	}
}


#include "rtslam/dataManagerMarkerFinderAbstract.impl.hpp"

#endif /* DATAMANAGERMARKERFINDERABSTRACT_HPP_ */
