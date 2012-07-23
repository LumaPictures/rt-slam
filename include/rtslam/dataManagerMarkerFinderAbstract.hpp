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
			int id;
			// Jafar standard translation/quat - ie tx,ty,tz,qw,qx,qy,qz
			jblas::vec7 pose;

			ublas::vector_range<jblas::vec7>
			translation()
			{
				return subrange(pose, 0,3);
			}

			ublas::vector_range<jblas::vec7>
			quaternion()
			{
				return subrange(pose, 3,7);
			}
		};

		typedef boost::shared_ptr<Marker> MarkerPtr;
		typedef std::deque<MarkerPtr> MarkerList;
		typedef std::map<int, MarkerList > IdMarkerMap;

		/**
		This class implements marker / tag searches

		@ingroup rtslam
		*/
		template<class RawSpec, class SensorSpec>
		class DataManagerMarkerFinderAbstract: public DataManagerAbstract, public SpecificChildOf<SensorSpec> {
			public:
				// Define the function linkToParentSensorSpec.
				ENABLE_LINK_TO_SPECIFIC_PARENT(SensorExteroAbstract, SensorSpec, SensorSpec, DataManagerAbstract);
				// Define the functions sensorSpec() and sensorSpecPtr().
				ENABLE_ACCESS_TO_SPECIFIC_PARENT(SensorSpec, sensorSpec);

			public: // public interface
				static const size_t DEFAULT_MAX_MARKERS = 10;
				DataManagerMarkerFinderAbstract(size_t maxMarkersPerId_=DEFAULT_MAX_MARKERS);

				void processKnown(raw_ptr_t data, double date_limit = -1.)
				{}

				void detectNew(raw_ptr_t data);

				// TODO: eventually, support multiple-marker tracking, and
				// convert this over to return a Marker list
				virtual MarkerPtr detectMarker(raw_ptr_t data) = 0;

			protected: // main data members
				IdMarkerMap markerMap;
				size_t maxMarkersPerId;

			protected: // parameters

			public: // getters and setters

			protected: // helper functions
				void addMarker(MarkerPtr newMarker);

		};

	}
}


#include "rtslam/dataManagerMarkerFinderAbstract.impl.hpp"

#endif /* DATAMANAGERMARKERFINDERABSTRACT_HPP_ */
