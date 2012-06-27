/**
 * \file visibilityMap.cpp
 * \date 23/12/2010
 * \author croussil
 * \ingroup rtslam
 */

#include "jmath/angle.hpp"
#include "rtslam/visibilityMap.hpp"
#include "rtslam/observationAbstract.hpp"



namespace jafar {
namespace rtslam {

	std::ostream& operator <<(std::ostream & s, VisibilityMap::Cell const & cell)
	{
		s << "| succ: " << cell.nSuccess << " | fail: " << cell.nFailure << 
			" | lastres: " << cell.lastResult << " | lastframe: " << cell.lastTryFrame << " |";
		return s;
	}
	
	std::ostream& operator <<(std::ostream & s, VisibilityMap const & vismap)
	{
		s << "| vis: " << vismap.lastVis << " | visUncert: " << vismap.lastVisUncert << " | cells: " << vismap.grid.map.size() << " |";
		if (vismap.lastCell) s << *vismap.lastCell;
		return s;
	}

	VisibilityMap::VisibilityMap(double angularRes, double distInit, double distFactor, int nDist, int nCertainty):
		grid(angularRes, distInit, distFactor, nDist, 1.0),
		nCertainty(nCertainty), lastCell(NULL), lastVis(0.5), lastVisUncert(0.0)
	{}
	
	VisibilityMap::VisibilityMap():
		grid(10.0, 3.0, 3.0, 4, 1.0),
		nCertainty(10), lastCell(NULL), lastVis(0.5), lastVisUncert(0.0)
	{}

	
	VisibilityMap::Cell* VisibilityMap::getCell(const observation_ptr_t obsPtr, bool create)
	{
		jblas::vec3 posObs = ublas::subrange(obsPtr->landmarkPtr()->reparametrized(), 0, 3); // FIXME should get some average position instead eg for segments
		jblas::vec3 posSen = ublas::subrange(obsPtr->sensorPtr()->globalPose(), 0, 3);
		return grid.getCell(posSen-posObs, create);
	}
	
	
	void VisibilityMap::addObservation(const observation_ptr_t obsPtr)
	{
		if (obsPtr->events.measured)
		{
			Cell &cell = *(getCell(obsPtr, true));
			cell.lastTryFrame = SPTR_CAST<SensorExteroAbstract>(obsPtr->sensorPtr())->rawCounter;
			cell.lastResult = obsPtr->events.updated;
			if (cell.lastResult) cell.nSuccess++; else cell.nFailure++;
			lastCell = &cell;
		}
	}
	
	
	void VisibilityMap::estimateVisibility(const observation_ptr_t obsPtr, double &visibility, double &certainty)
	{
		Cell *cell = getCell(obsPtr, false);
		if (!cell) cell = lastCell;
		if (!cell) { lastVis = visibility = 0.5; lastVisUncert = certainty = 0.;  return; }
		if (cell->lastResult) { lastVis = visibility = 1.; lastVisUncert = certainty = 1.; return; }
		int nTries = cell->nSuccess + cell->nFailure;
		double rate = cell->nSuccess / (double)nTries;
		
		lastVis = visibility = rate;
		lastVisUncert = certainty = (nTries >= nCertainty ? 1.0 : nTries/(double)nCertainty);
	}



}}

