/**
 * \file dataManagerMarkerFinder.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#ifndef DATAMANAGERMARKERFINDER_HPP_
#define DATAMANAGERMARKERFINDER_HPP_

#include "boost/shared_ptr.hpp"

#include "rtslam/dataManagerAbstract.hpp"

namespace jafar {
	namespace rtslam {

		/**
		This class holds data representing a single marker

		@ingroup rtslam
		*/
		struct Marker
		{
				size_t id;
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

		/**
		This class implements marker / tag searches

		@ingroup rtslam
		*/
		template<class RawSpec, class SensorSpec>
		class DataManagerMarkerFinder: public DataManagerAbstract, public SpecificChildOf<SensorSpec> {

			public:
				// Define the function linkToParentSensorSpec.
				ENABLE_LINK_TO_SPECIFIC_PARENT(SensorExteroAbstract, SensorSpec, SensorSpec, DataManagerAbstract);
				// Define the functions sensorSpec() and sensorSpecPtr().
				ENABLE_ACCESS_TO_SPECIFIC_PARENT(SensorSpec, sensorSpec);

			public: // public interface
				DataManagerMarkerFinder()
				{
				}
				virtual ~DataManagerMarkerFinder() {
				}
				void processKnown(raw_ptr_t data, double date_limit = -1.) {
				}
				void detectNew(raw_ptr_t data);

				// TODO: eventually, support multiple-marker tracking, and
				// convert this over to return a Marker list
				virtual MarkerPtr detectMarker(raw_ptr_t data);

			protected: // main data members

			protected: // parameters

			public: // getters and setters

			protected: // helper functions

		};

	}
}


#include "rtslam/dataManagerMarkerFinder.impl.hpp"

#endif /* DATAMANAGERMARKERFINDER_HPP_ */
