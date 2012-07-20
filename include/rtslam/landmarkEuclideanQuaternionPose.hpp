/**
 * \file landmarkEuclideanQuaternionPose.hpp
 *
 * \date 20/08/2012
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#ifndef LANDMARKEUCLIDEANQUATERIONPOSE_HPP_
#define LANDMARKEUCLIDEANQUATERIONPOSE_HPP_

#include "boost/shared_ptr.hpp"
#include "rtslam/landmarkAbstract.hpp"
#include "rtslam/quatTools.hpp"
//#include "rtslam/ahpTools.hpp"

/**
 * General namespace for Jafar environment.
 * \ingroup jafar
 */
namespace jafar {
	namespace rtslam {

		class LandmarkEuclideanQuaternionPose;
		typedef boost::shared_ptr<LandmarkEuclideanQuaternionPose> ptquat_ptr_t;



		/**
		 * Class for for translation + rotation
		 * \ingroup rtslam
		 */
		class LandmarkEuclideanQuaternionPose: public LandmarkAbstract {
			public:


				/**
				 * Constructor from map
				 */
				LandmarkEuclideanQuaternionPose(const map_ptr_t & mapPtr);

				/**
				 * Constructor for simulated landmark.
				 */
				LandmarkEuclideanQuaternionPose(const simulation_t dummy, const map_ptr_t & mapPtr);
				/**
				 * Constructor by replacement: occupied the same filter state as a specified previous lmk. _icomp is the complementary memory, to be relaxed by the user.
				 */
				LandmarkEuclideanQuaternionPose(const map_ptr_t & _mapPtr, const landmark_ptr_t _prevLmk,jblas::ind_array & _icomp);

				virtual ~LandmarkEuclideanQuaternionPose() {
//					cout << "Deleted landmark: " << id() << ": " << typeName() << endl;
				}

				virtual std::string typeName() const {
					return "Euclidean-Quaternion Pose";
				}


				static size_t size(void) {
					return 7;
				}

				virtual size_t mySize() {return size();}

				virtual size_t reparamSize() {return size();}

				virtual jblas::vec3 center() { return ublas::subrange(state.x(), 0, 3); }

				virtual double uncertainty()
				{
					// max uncertainty of each component is a fair approximation
					double max_uncert = 0.;
					for(size_t i = 0; i < size(); ++i)
					{
						double uncert = state.P()(i,i);
						if (uncert > max_uncert) max_uncert = uncert;
					}
					return 3 * sqrt(max_uncert);
				}

				virtual vec reparametrize_func(const vec & lmk) const {
					return lmk;
				}

				void reparametrize_func(const vec & lmk, vec & lnew, mat & LNEW_lmk) const {
					lnew = lmk;
					LNEW_lmk = identity_mat(size());
				}

				virtual bool needToReparametrize(){
					return false; // never reparametrize a EuclideanQuaternionPose
				}
		}; // class LandmarkEuclideanQuaternionPose
	} // namespace rtslam
} // namespace jafar


#endif /* LANDMARKEUCLIDEANQUATERIONPOSE_HPP_ */
