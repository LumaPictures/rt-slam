/**
 * \file dataManagerMarkerFinderAruco.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#ifndef DATAMANAGERMARKERFINDERARUCO_HPP_
#define DATAMANAGERMARKERFINDERARUCO_HPP_

#include "boost/shared_ptr.hpp"

#include "rtslam/dataManagerMarkerFinderAbstract.hpp"

namespace jafar {
	namespace rtslam {

		/**
		Implementation of DataManagerMarkerFinderAbstruct using the aruco marker-finder library

		@ingroup rtslam
		*/
		template<class RawSpec, class SensorSpec>
		class DataManagerMarkerFinderAruco : public DataManagerMarkerFinderAbstract<RawSpec, SensorSpec>
		{
			public: // public interface
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

#include "rtslam/dataManagerMarkerFinderAruco.impl.hpp"

#endif /* DATAMANAGERMARKERFINDERARUCO_HPP_ */
