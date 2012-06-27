/**
 * \file visibilityMap.hpp
 * \date 23/12/2010
 * \author croussil
 *
 * File defining the VisibilityMap, an object that keeps records
 * of places from where a landmark is or is not visible.
 * \ingroup rtslam
 */

#ifndef VISIBILITY_MANAGER_HPP_
#define VISIBILITY_MANAGER_HPP_

#include <map>
#include "jmath/misc.hpp"
#include "jmath/jblas.hpp"
#include "rtslam/rtSlam.hpp"
#include "rtslam/spaceGrid.hpp"

namespace jafar {
namespace rtslam {

	/**
	 * This class is a map that keeps records of places frome where a landmark
	 * was or was not visible, in order to avoid to repeatedly search a
	 * landmark from a place it cannot be seen.
	 */
	class VisibilityMap
	{
		protected:
			struct Cell
			{
				unsigned nSuccess;
				unsigned nFailure;
				bool lastResult;
				unsigned lastTryFrame;
				Cell(): nSuccess(0), nFailure(0), lastResult(false), lastTryFrame(0) {}
			};
			SphericalGrid<Cell> grid;
			double nCertainty;
			Cell *lastCell;
			double lastVis, lastVisUncert;
		protected:
			Cell* getCell(const observation_ptr_t obsPtr, bool create = false);
		public:
			/**
			 * Constructor
			 * @param angularRes the angular resolution of the map (yaw and pitch)
			 * @param distInit,distFactor,nDist borders between the depth cells are distInit,distInit*distFactor,...,distInit*distFactor^(nDist-2) -> nDist depth cells
			 * @param nCertainty a visibility score is supposed to be completely certain if at least nCertainty observations were used to compute it
			 */
			VisibilityMap(double angularRes, double distInit, double distFactor, int nDist, int nCertainty);
			VisibilityMap();
			/**
			 * add an observation to the map
			 */
			void addObservation(const observation_ptr_t obsPtr);
			/**
			 * return the score of visibility and its certainty (both beween 0 and 1)
			 */
			void estimateVisibility(const observation_ptr_t obsPtr, double &visibility, double &certainty);
			
			friend std::ostream& operator <<(std::ostream & s, Cell const & cell);
			friend std::ostream& operator <<(std::ostream & s, VisibilityMap const & vismap);
	};
	




}}

#endif
