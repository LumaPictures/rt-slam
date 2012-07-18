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
			if (mapManagerPtr()->mapSpaceForInit())
			{
				// 2a. Create the lmk and associated obs object.
				observation_ptr_t obsPtr =
						mapManagerPtr()->createNewLandmark(shared_from_this());
			}
		}


	} // namespace ::rtslam
} // namespace jafar::

