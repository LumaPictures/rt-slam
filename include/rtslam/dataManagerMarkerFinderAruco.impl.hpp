/**
 * \file dataManagerMarkerFinderAruco.cpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#include "rtslam/dataManagerMarkerFinderAruco.hpp"
#include "boost/make_shared.hpp"

using namespace boost;

namespace jafar {
	namespace rtslam {

		template<class RawSpec, class SensorSpec>
		MarkerPtr DataManagerMarkerFinderAruco<RawSpec, SensorSpec>::
		detectMarker(raw_ptr_t data)
		{
			return make_shared<Marker>();
		}


	} // namespace ::rtslam
} // namespace jafar::

