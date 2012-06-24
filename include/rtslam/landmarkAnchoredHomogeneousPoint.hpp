/**
 * \file landmarkAnchoredHomogeneousPoint.hpp
 * \author jsola
 * \date 14/02/2010
 *
 * Header file for anchored homogeneous point
 * \ingroup rtslam
 */

#ifndef LANDMARKANCHOREDHOMOGENEOUSPOINT_HPP_
#define LANDMARKANCHOREDHOMOGENEOUSPOINT_HPP_

#include "boost/shared_ptr.hpp"
#include "rtslam/landmarkAbstract.hpp"
#include "rtslam/quatTools.hpp"
#include "rtslam/ahpTools.hpp"
#include "rtslam/landmarkEuclideanPoint.hpp"

/**
 * General namespace for Jafar environment.
 * \ingroup jafar
 */
namespace jafar {
	namespace rtslam {

		class LandmarkAnchoredHomogeneousPoint;
		typedef boost::shared_ptr<LandmarkAnchoredHomogeneousPoint> ahp_ptr_t;



		/**
		 * Class for anchored homogeneous 3D points
		 * \ingroup rtslam
		 */
		class LandmarkAnchoredHomogeneousPoint: public LandmarkAbstract {
			public:


				/**
				 * Constructor from map
				 */
				LandmarkAnchoredHomogeneousPoint(const map_ptr_t & mapPtr);
				LandmarkAnchoredHomogeneousPoint(const simulation_t dummy, const map_ptr_t & mapPtr);

				virtual ~LandmarkAnchoredHomogeneousPoint() {
//					std::cout << "Deleted landmark: " << id() << ": " << typeName() << std::endl;
				}

				virtual std::string typeName() const {
					return "Anchored-homogeneous-point";
				}


				static size_t size(void) {
					return 7;
				}


				virtual bool needToDie();

				virtual size_t mySize() {return size();}

				virtual size_t reparamSize() {return LandmarkEuclideanPoint::size();}

				virtual jblas::vec3 center() { return reparametrize_func(state.x()); }

				virtual double uncertainty()
				{
					// max uncertainty of an AHP will almost always be depth uncertainty
					double id = state.x()(6);
					double ID = 3. * sqrt(state.P()(6,6));
					if (id-ID <= 0) return 1e6;
										 else return (1./(id-ID) - 1./id);
				}

				virtual vec reparametrize_func(const vec & lmk) const {
					return lmkAHP::ahp2euc(lmk);
				}

				/**
				 * Reparametrize to Euclidean, with Jacobians.
				 * \param ahp the anchored homogeneous point to be reparametrized.
				 * \param euc the returned Euclidean point.
				 * \param EUC_ahp the Jacobian of the conversion.
				 */
				void reparametrize_func(const vec & ahp, vec & euc, mat & EUC_ahp) const {
					lmkAHP::ahp2euc(ahp, euc, EUC_ahp);
				}

		}; // class LandmarkAnchoredHomogeneousPoint


	} // namespace rtslam
} // namespace jafar

#endif /* LANDMARKANCHOREDHOMOGENEOUSPOINT_HPP_ */
