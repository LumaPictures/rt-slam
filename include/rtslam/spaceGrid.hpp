/**
 * \file spaceGrid.hpp
 *
 * Header file
 *
 * \date 21/06/2012
 * \author croussil
 *
 * \ingroup rtslam
 */

#ifndef SPACEGRID_HPP_
#define SPACEGRID_HPP_


#include <vector>
#include <map>
#include "kernel/jafarDebug.hpp"
#include "jmath/jblas.hpp"
#include "jmath/misc.hpp"
#include "jmath/angle.hpp"


namespace jafar {
namespace rtslam {
/*
	template<class Cell>
	class Grid
	{
		public:
			Cell* getCell(jblas::vec3 pos, bool create = false);
	};
*/

	/**
	 * A grid in spherical coordinates theta,phi,r with dynamic slicing of theta
	 * according to phi so that all (theta,phi) cells have approximately the same
	 * area (typically only one at the poles).
	 * Slicing of r and phi can also optionally be dynamic with a geometric
	 * progression wrt r.
	 */
	template<class Cell>
	class SphericalGrid
	{
		protected:
			struct Index
			{
				int theta, phi, r;
				Index(int theta, int phi, int r): theta(theta), phi(phi), r(r) {}
				bool operator<(Index const & a) const
					{ return (theta == a.theta ? (phi == a.phi ? r < a.r : phi < a.phi) : theta < a.theta); }
			};

			double angularRes;
			double distInit;
			double distFactor;
			int nDist;
			double phiFactor;

		public:
			std::map<Index, Cell> map;
			typedef typename std::map<Index, Cell>::iterator iterator;
		public:

			SphericalGrid(double angularRes, double distInit, double distFactor, int nDist, double phiFactor);

			void rebuild(double angularRes, double distInit, double distFactor, int nDist, double phiFactor);
			Cell* getCell(jblas::vec3 pos, bool create = false);
			void downRes(double factor);

	};




	//*******************************************************************************



	template <class Cell>
	SphericalGrid<Cell>::SphericalGrid(double angularRes, double distInit, double distFactor, int nDist, double phiFactor)
	{
		rebuild(angularRes, distInit, distFactor, nDist, phiFactor);
	}


	template <class Cell>
	void SphericalGrid<Cell>::rebuild(double angularRes, double distInit, double distFactor, int nDist, double phiFactor)
	{
		this->angularRes = angularRes;
		this->distInit = distInit;
		this->distFactor = distFactor;
		this->nDist = nDist;
		this->phiFactor = phiFactor;

		// TODO precompute phi_size_of_dist in vector and theta_size_of_phi in matrix
	}


	template <class Cell>
	Cell* SphericalGrid<Cell>::getCell(jblas::vec3 pos, bool create)
	{
		double theta = jmath::radToDeg(std::atan2(pos(0), pos(1)));
		double phi = jmath::radToDeg(std::atan2(pos(2), ublas::norm_2(ublas::subrange(pos,0,2))));
		double r = ublas::norm_2(pos);

		// r_i
		double ang_res = angularRes;
		int r_i = nDist-1;
		double dist = distInit;
		for(int i = 0; i < nDist-1; ++i) { if (r < dist) { r_i = i; break; } dist *= distFactor; ang_res *= phiFactor; }
		// phi_i : use ang_res/2 at poles for better repartition
		int phi_size = jmath::round(180./ang_res);
		if (phi_size < 1) phi_size = 1;
		ang_res = 180./phi_size;
		int phi_i = (int)((phi+90.)/ang_res+0.5);
		// theta_i
		double phi1 = -90.+((double)(phi_size/2)-0.5)*ang_res;
		double phi2 = -90.+((double)(phi_size/2)+0.5)*ang_res;
		double ref_area = 2*M_PI*(sin(phi2*M_PI/180.)-sin(phi1*M_PI/180.)) / (phi_size*2); // cell area at the equator
		phi1 = (phi_i == 0 ? -90. : -90.+((double)(phi_i)-0.5)*ang_res);
		phi2 = (phi_i == phi_size ? 90. : -90.+((double)(phi_i)+0.5)*ang_res);
		int theta_size = jmath::round(2*M_PI*(sin(phi2*M_PI/180.)-sin(phi1*M_PI/180.)) / ref_area);
		if (theta_size < 1) theta_size = 1;
		ang_res = 360./theta_size;
		int theta_i = (int)((theta+180.-1e-6)/ang_res);

//std::cout << "getCell params " << angularRes << "," << distInit << "," << distFactor << "," << nDist << "," << phiFactor << " ; pos " << pos << " ; bin (tpr) " << theta_i << "," << phi_i << "," << r_i << std::endl;
		JFR_ASSERT(r_i >= 0 && r_i < nDist, "r_i out of bounds " << r_i << " | " << nDist);
		JFR_ASSERT(phi_i >= 0 && phi_i < phi_size+1, "phi_i out of bounds " << phi_i << " | " << phi_size+1);
		JFR_ASSERT(theta_i >= 0 && theta_i < theta_size, "theta_i out of bounds " << theta_i << " | " << theta_size);

		if (create)
			return &(map[Index(theta_i, phi_i, r_i)]);
		else
		{
			typename std::map<Index,Cell>::iterator it = map.find(Index(theta_i, phi_i, r_i));
			return (it == map.end() ? NULL : &(it->second));
		}
	}


	template <class Cell>
	void SphericalGrid<Cell>::downRes(double factor)
	{
		rebuild(angularRes*factor, distInit*factor, distFactor*factor, nDist, phiFactor*factor);
	}


}}

#endif
