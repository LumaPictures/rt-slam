/**
 * \file mapManagerAbstract.cpp
 * \date 10/03/2010
 * \author nmansard
 * \ingroup rtslam
 */

#include <boost/shared_ptr.hpp>

#include "rtslam/rtSlam.hpp"
#include "rtslam/mapManager.hpp"
#include "rtslam/landmarkAbstract.hpp"
#include "rtslam/observationFactory.hpp"
#include "rtslam/observationAbstract.hpp"
#include "rtslam/dataManagerAbstract.hpp"

namespace jafar {
	namespace rtslam {
		using namespace std;

		/** ***************************************************************************************
			MapManagerAbstract
		******************************************************************************************/
	
		observation_ptr_t MapManagerAbstract::createNewLandmark(data_manager_ptr_t dmaOrigin)
		{
			landmark_ptr_t newLmk = lmkFactory->createInit(mapPtr());
			newLmk->setId();
			newLmk->linkToParentMapManager(shared_from_this());
			observation_ptr_t resObs;

			for (MapManagerAbstract::DataManagerList::iterator
			     iterDMA = dataManagerList().begin();
			     iterDMA != dataManagerList().end(); ++iterDMA)
			{
				data_manager_ptr_t dma = *iterDMA;
				observation_ptr_t newObs =
				    dma->observationFactory()->create(dma->sensorPtr(), newLmk);

				/* Insert the observation in the graph. */
				newObs->linkToParentDataManager(dma);
				newObs->linkToParentLandmark(newLmk);
				newObs->linkToSensor(dma->sensorPtr());
				newObs->linkToSensorSpecific(dma->sensorPtr());
				newObs->setId();

				/* Store for the return the obs corresponding to the dma origin. */
				if (dma == dmaOrigin) resObs = newObs;
			}
			
			return resObs;
		}

	  void MapManagerAbstract::unregisterLandmark(landmark_ptr_t lmkPtr, bool liberateFilter)
		{
			// first unlink all observations
			for (LandmarkAbstract::ObservationList::iterator
			     obsIter = lmkPtr->observationList().begin();
			     obsIter != lmkPtr->observationList().end(); ++obsIter)
			{
				observation_ptr_t obsPtr = *obsIter;
				obsPtr->dataManagerPtr()->unregisterChild(obsPtr);
			}
			// liberate map space
			if( liberateFilter )
			  mapPtr()->liberateStates(lmkPtr->state.ia());
			// now unlink landmark
			ParentOf<LandmarkAbstract>::unregisterChild(lmkPtr);
		}



    void MapManagerAbstract::reparametrizeLandmark(landmark_ptr_t lmkinit)
		{
			//cout<<__PRETTY_FUNCTION__<<"(#"<<__LINE__<<"): " <<"" << endl;

			// unregister lmk
			unregisterLandmark(lmkinit, false);
			//lmkinit->destroyDisplay(); // cannot do that here, display is using it...

			// Create a new landmark advanced instead of the previous init lmk.
			jblas::ind_array idxComp(lmkFactory->sizeComplement());
			//cout << __PRETTY_FUNCTION__ << "about to create lmkcv." << endl;
			landmark_ptr_t lmkconv = lmkFactory->createConverged(mapPtr(), lmkinit, idxComp);

			// link new landmark
			lmkconv->linkToParentMapManager(shared_from_this());

			// Algebra: 2a. compute the jac and 2b. update the filter.
			// a. Call reparametrize_func()
			const size_t size_init = lmkinit->mySize();
			const size_t size_conv = lmkconv->mySize();
			mat CONV_init(size_conv, size_init);
			vec sinit = lmkinit->state.x();
			vec sconv(size_conv);
			//cout << __PRETTY_FUNCTION__ << "about to call lmk->reparametrize_func()" << endl;
			lmkinit->reparametrize_func(sinit, sconv, CONV_init);

			// b. Call filter->reparametrize().
			//cout << __PRETTY_FUNCTION__ << "about to call filter->reparametrize()" << endl;
			lmkconv->state.x() = sconv;
			mapPtr()->filterPtr->reparametrize(mapPtr()->ia_used_states(),
				CONV_init, lmkinit->state.ia(), lmkconv->state.ia());

			// Transfer info from the old lmk to the new one.
			//cout << __PRETTY_FUNCTION__ << "about to transfer lmk info." << endl;
			lmkconv->transferInfoLmk(lmkinit);

			// Create the cv-lmk set of observations, one per sensor.
			for(LandmarkAbstract::ObservationList::iterator
			    obsIter = lmkinit->observationList().begin();
			    obsIter != lmkinit->observationList().end(); ++obsIter)
			{
				observation_ptr_t obsinit = *obsIter;
				data_manager_ptr_t dma = obsinit->dataManagerPtr();
				sensor_ptr_t sen = obsinit->sensorPtr();

				//cout << __PRETTY_FUNCTION__ << "about to create new obs" << endl;
				observation_ptr_t obsconv = dma->observationFactory()->create(sen, lmkconv);
				obsconv->linkToParentDataManager(dma);
				obsconv->linkToParentLandmark(lmkconv);
				obsconv->linkToSensor(sen);
				obsconv->linkToSensorSpecific(sen);
				// transfer info to new obs
				obsconv->transferInfoObs(obsinit);
			}

			// liberate unused map space.
			mapPtr()->liberateStates(idxComp);
		}
		
		
		/** ***************************************************************************************
			MapManager
		******************************************************************************************/
		
		void MapManager::manageDefaultDeletion()
		{
			for(LandmarkList::iterator lmkIter = landmarkList().begin();
					 lmkIter != landmarkList().end(); ++lmkIter)
			{
				landmark_ptr_t lmkPtr = *lmkIter;
				bool needToDie = lmkPtr->needToDie();
				if (!needToDie)
				for(LandmarkAbstract::ObservationList::iterator obsIter = lmkPtr->observationList().begin();
						obsIter != lmkPtr->observationList().end(); ++obsIter)
				{ // all observations (sensors) must agree to delete a landmark
					observation_ptr_t obsPtr = *obsIter;

					JFR_ASSERT(obsPtr->counters.nMatch <= obsPtr->counters.nSearch, "counters.nMatch " 
										 << obsPtr->counters.nMatch << " > counters.nSearch " << obsPtr->counters.nSearch);
					JFR_ASSERT(obsPtr->counters.nInlier <= obsPtr->counters.nMatch, "counters.nInlier " 
										 << obsPtr->counters.nInlier << " > counters.nMatch " << obsPtr->counters.nMatch);

					// kill if any sensor has search area too large
					if (obsPtr->events.predicted && obsPtr->events.measured && !obsPtr->events.updated)
					{
						if (obsPtr->searchSize > killSizeTh) {
							JFR_DEBUG( "Lmk " << lmkPtr->id() << " Killed by size (size " << obsPtr->searchSize << ")" );
							needToDie = true;
							break;
						}
					}
				}
				if (needToDie)
				{
					lmkIter = unregisterLandmark(lmkIter);
				}
			}
		}
		
		void MapManager::manageReparametrization()
		{
			for(LandmarkList::iterator lmkIter = landmarkList().begin();
					 lmkIter != landmarkList().end(); ++lmkIter)
			{
				landmark_ptr_t lmkPtr = *lmkIter;
				if (lmkPtr->converged) continue; // already reparametrized, we don't go back for now
				bool needToReparametrize = true;
				bool hasObserved = false;
				for(LandmarkAbstract::ObservationList::iterator obsIter = lmkPtr->observationList().begin();
						obsIter != lmkPtr->observationList().end(); ++obsIter)
				{ // all observations (sensors) must agree to reparametrize a landmark, and at least one must have observed it
					observation_ptr_t obsPtr = *obsIter;
					if (obsPtr->events.updated) hasObserved = true;
					if (obsPtr->computeLinearityScore() > reparTh)
					{
						needToReparametrize = false;
						break;
					}
				}
				if (hasObserved && needToReparametrize)
					lmkIter = reparametrizeLandmark(lmkIter);
			}
		}

		
		/** ***************************************************************************************
			MapManagerOdometry
		******************************************************************************************/
		
		void MapManagerOdometry::manageDeletion()
		{
			for(MapManagerAbstract::LandmarkList::iterator lmkIter = this->landmarkList().begin();
					 lmkIter != this->landmarkList().end(); ++lmkIter)
			{
				landmark_ptr_t lmkPtr = *lmkIter;
				bool needToDie = true;
				for(LandmarkAbstract::ObservationList::iterator obsIter = lmkPtr->observationList().begin();
						obsIter != lmkPtr->observationList().end(); ++obsIter)
				{ // all observations (sensors) must agree to delete a landmark
					observation_ptr_t obsPtr = *obsIter;

					// kill if all sensors have unstable and inconsistent observations
					if (!(obsPtr->counters.nFrameSinceLastVisible > 0 ||
//					    (obsPtr->counters.nInlier == 1 && !obsPtr->events.updated) ||
					    obsPtr->counters.nSearchSinceLastInlier > 1))
					{
						needToDie = false;
						break;
					}
				}
				if (needToDie)
					lmkIter = unregisterLandmark(lmkIter);
			}
		}
		
		bool MapManagerOdometry::isExclusive(observation_ptr_t obsPtr)
		{
			return (!(obsPtr->counters.nSearchSinceLastInlier > 1));
		}
		
		/** ***************************************************************************************
			MapManagerGlobal
		******************************************************************************************/

		void MapManagerGlobal::manageDeletion()
		{
			// 1st, check individual parameters
			/*
			Kill if too unstable, invisible and too young, invisible and too uncertain
			*/
			for(MapManagerAbstract::LandmarkList::iterator lmkIter = this->landmarkList().begin();
					 lmkIter != this->landmarkList().end(); ++lmkIter)
			{
				landmark_ptr_t lmkPtr = *lmkIter;
				lmkPtr->fillEvents();
				// 1a. test stability
				bool needToDie_unstable = true, isYoung = true;
				for(LandmarkAbstract::ObservationList::iterator obsIter = lmkPtr->observationList().begin();
						obsIter != lmkPtr->observationList().end(); ++obsIter)
				{ // all observations (sensors) must agree to delete a landmark
					observation_ptr_t obsPtr = *obsIter;

					// kill if all sensors have unstable and inconsistent observations
					if (obsPtr->counters.nSearch > killSearchTh)
					{
						isYoung = false;
						if (needToDie_unstable)
						{
							double matchRatio = obsPtr->counters.nMatch / (double)obsPtr->counters.nSearch;
							double consistencyRatio = obsPtr->counters.nInlier / (double)obsPtr->counters.nMatch;

							if (matchRatio >= killMatchTh && consistencyRatio >= killConsistencyTh)
								needToDie_unstable = false;
						}
					}
				}
				if (isYoung) needToDie_unstable = false;
				// 1b. test age
				bool needToDie_young = isYoung && !lmkPtr->visible;

				// 1c. test uncertainty
				bool needToDie_uncertain = false;
				if (!lmkPtr->visible && !(needToDie_unstable || needToDie_young))
				{
					jblas::vec rob2lmk = lmkPtr->center() - ublas::subrange(this->mapPtr()->robotList().front()->state.x(), 0,3); // we will always have only one robot
					double dist = ublas::norm_2(rob2lmk);
					if (dist < 10.) needToDie_uncertain = (lmkPtr->uncertainty() / dist > killUncertaintyTh);
				}

				// 1z. delete if necessary
				if (needToDie_unstable || needToDie_young || needToDie_uncertain)
				{
					JFR_DEBUG( "Lmk " << lmkPtr->id() << " Killed because " << (needToDie_unstable ? "unstable" : (needToDie_young ? "young" : (needToDie_uncertain ? "uncertain" : "?"))));
					lmkIter = unregisterLandmark(lmkIter);
				}
			}

			// 2nd, check density
			/*
			Build spherical 3D grids to limit landmarks density.
			Only keep one invisible landmark per cell, one matched landmark per cell,
			and one visible but failed landmark per cell. The landmark with lowest
			uncertainty is kept (it implies that it has been observed from different
			places).
			All three grids are initialized coherently with the 2D active search grids
			of the different sensors, but the grid for invisible landmarks has its
			resolution progressively reduced when we need room for new landmarks.
			*/
			fillGrids(true);
			processGrid(grid_visible_updated);
			processGrid(grid_visible_failed);
			processGrid(grid_invisible);
		}

		
		void MapManagerGlobal::fillGrids(bool fill_visible)
		{
			if (fill_visible) {
				grid_visible_updated.map.clear();
				grid_visible_failed.map.clear();
			}
			grid_invisible.map.clear();
			for(MapManagerAbstract::LandmarkList::iterator lmkIter = this->landmarkList().begin();
					 lmkIter != this->landmarkList().end(); ++lmkIter)
			{
				landmark_ptr_t lmkPtr = *lmkIter;
				jblas::vec rob2lmk = lmkPtr->center() - ublas::subrange(this->mapPtr()->robotList().front()->state.x(), 0,3); // we will always have only one robot
				if ((*lmkIter)->visible)
				{
					if (fill_visible)
					{
						if ((*lmkIter)->updatable) grid_visible_updated.getCell(rob2lmk, true)->landmarks.push_back(lmkPtr);
																	else grid_visible_failed  .getCell(rob2lmk, true)->landmarks.push_back(lmkPtr);
					}
				} else
					grid_invisible.getCell(rob2lmk, true)->landmarks.push_back(lmkPtr);
			}
		}


		void MapManagerGlobal::processGrid(SphericalGrid<Cell> & grid)
		{
			for(SphericalGrid<Cell>::iterator gridIter = grid.map.begin(); gridIter != grid.map.end(); ++gridIter)
				if (gridIter->second.landmarks.size() > 1)
				{
					// TODO check with visibility map, if landmarks have few in common we could keep them both

					// find best
					double min_uncert = 1e12;
					landmark_ptr_t min_lmk;
					for(std::list<landmark_ptr_t>::iterator lmkIter = gridIter->second.landmarks.begin(); lmkIter != gridIter->second.landmarks.end(); ++lmkIter)
					{
						double uncert = (*lmkIter)->uncertainty();
						if (uncert < min_uncert)
							{ min_uncert = uncert; min_lmk = *lmkIter; }
						else if (uncert == min_uncert) // may happen with close to infinite uncertainty
						{
							if ((*lmkIter)->countObserved() > min_lmk->countObserved())
								min_lmk = *lmkIter;
						}
					}

					// kill others
					bool erase_it;
					for(std::list<landmark_ptr_t>::iterator lmkIter = gridIter->second.landmarks.begin();
							lmkIter != gridIter->second.landmarks.end(); erase_it ? lmkIter = gridIter->second.landmarks.erase(lmkIter) : ++lmkIter)
					{
						erase_it = false;
						if (*lmkIter != min_lmk)
						{
							JFR_DEBUG( "Lmk " << (*lmkIter)->id() << " Killed by density");
							unregisterLandmark(*lmkIter);
							erase_it = true;
						}
					}
				}
		}


		bool MapManagerGlobal::mapSpaceForInit()
		{
			// FIXME should loop over all map managers ?
			const double downresFactor = 1.1;

			bool space;
			while (!(space = MapManager::mapSpaceForInit()))
			{
				JFR_DEBUG("down res invisible map to free space");
				grid_invisible.downRes(downresFactor);
				fillGrids(false);
				processGrid(grid_invisible);
				if (grid_invisible.map.size() <= 1) break;
			}
			return space;
		}
		

		bool MapManagerGlobal::isExclusive(observation_ptr_t obsPtr)
		{
//if (!obsPtr->updatable) std::cout << "obs " << obsPtr->id() << " is not exclusive" << std::endl;
			return obsPtr->updatable;
		}


        /** ***************************************************************************************
            MapManagerRecent
        ******************************************************************************************/

        bool MapManagerRecent::mapSpaceForInit()
        {
            while (!MapManagerAbstract::mapSpaceForInit() && mapPtr()->current_size
                    && !landmarkList().empty())
            {
                deleteOldest();
            }
            JFR_ASSERT(MapManagerAbstract::mapSpaceForInit(), "insufficient space for new landmark, even after clearing out map");
            return true;
        }

        void MapManagerRecent::deleteOldest()
        {
            // the most recent landmarks are always added at the back, so the
            // oldest should be at the front!
        	unregisterLandmark(landmarkList().front());
        }


	}
}

