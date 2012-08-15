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
#include "rtslam/hardwareSensorAbstract.hpp"

namespace jafar {
	namespace rtslam {

		/**
		This class holds data representing a single marker

		@ingroup rtslam
		*/
		class Marker : public hardware::ReadingAbstract
		{
			public:
				typedef ublas::vector_range<jblas::vec> range_type;
				Marker(int id_=-1, CovType covType=ctNone) :
					ReadingAbstract(covType), id(id_)
				{
					addQuantity(qPos);
					addQuantity(qOriQuat);
					initData();
				}

				int id;

				range_type
				translation()
				{
					return getQuantityData(qPos);
				}

				range_type
				quaternion()
				{
					return getQuantityData(qOriQuat);
				}

				range_type
				pose()
				{
					return ublas::subrange(reading.data, getQuantity(qPos),
							getQuantity(qOriQuat) + QuantityDataSizes[qOriQuat]);
				}

			protected:
				void initData()
				{
					ReadingAbstract::initData();
					reading.data[getQuantity(qOriQuat) + 3] = 1;
				}
		};

		typedef boost::shared_ptr<Marker> MarkerPtr;

		class DataManagerMarkerFinderAbstract;
		typedef boost::shared_ptr<DataManagerMarkerFinderAbstract> data_man_markerfinder_ptr_t;

		/**
		This class implements marker / tag searches

		@ingroup rtslam
		*/
		class DataManagerMarkerFinderAbstract: public DataManagerAbstract {
			public: // public interface
				DataManagerMarkerFinderAbstract(float markerSize_);

				void processKnown(raw_ptr_t data, double date_limit = -1.)
				{}

				void detectNew(raw_ptr_t data);

				// TODO: eventually, support multiple-marker tracking, and
				// convert this over to return a Marker list
				virtual MarkerPtr detectMarker(raw_ptr_t data) = 0;

			protected: // main data members
				float markerSize;

			public: // getters and setters
				float getMarkerSize() { return markerSize; }

			protected: // helper functions
				void setSensorPoseFromMarker(MarkerPtr newMarker);

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

		// this func separated from setSensorPoseFromMarker to abstract
		// it, so that eventually code in sensorAbsloc can be converted to
		// use it (if/when, that is done, this func should probably be moved
		// in there)
		void updatePoseFromAbsReading(SensorAbstract& sensor, const hardware::ReadingAbstract& reading);
	}
}


#include "rtslam/dataManagerMarkerFinderAbstract.impl.hpp"

#endif /* DATAMANAGERMARKERFINDERABSTRACT_HPP_ */
