/**
 * \file dataManagerMarkerFinderAruco.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#ifndef DATAMANAGERMARKERFINDERARUCO_HPP_
#define DATAMANAGERMARKERFINDERARUCO_HPP_

#ifdef HAVE_ARUCO

#include "boost/shared_ptr.hpp"

#include "rtslam/dataManagerMarkerFinderAbstract.hpp"

#include "aruco.h"

namespace jafar {
	namespace rtslam {

		/**
		Implementation of DataManagerMarkerFinder using the aruco marker-finder library

		@ingroup rtslam
		*/
		template<class RawSpec, class SensorSpec>
		class DataManagerMarkerFinderAruco : public DataManagerMarkerFinder<RawSpec, SensorSpec>
		{
			public: // public interface
				DataManagerMarkerFinderAruco();

				// TODO: eventually, support multiple-marker tracking, and
				// convert this over to return a Marker list
				virtual MarkerPtr detectMarker(raw_ptr_t data);

			protected: // main data members
				aruco::CameraParameters camParams;
				float markerSize;
				aruco::MarkerDetector mDetector;

			protected: // parameters

			public: // getters and setters

			protected: // helper functions

		};

	}
}

#include "rtslam/dataManagerMarkerFinderAruco.impl.hpp"

#endif // HAVE_ARUCO

#endif // DATAMANAGERMARKERFINDERARUCO_HPP_
