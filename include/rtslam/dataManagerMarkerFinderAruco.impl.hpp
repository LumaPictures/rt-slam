/**
 * \file dataManagerMarkerFinderAruco.cpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#ifdef HAVE_ARUCO

#include <vector>
#include "rtslam/dataManagerMarkerFinderAruco.hpp"
#include "boost/make_shared.hpp"
#if CV_MAJOR_VERSION*10 + CV_MINOR_VERSION > 21
#include <opencv2/core/core.hpp>
#else
#include <cxcore.hpp> // opencv
#endif

namespace jafar {
	namespace rtslam {
		// Utility for converting an opencv Mat vector to vector type with
		// a known/set type (ie, a ublas/jblas vector)
		template<class TypedVec>
		inline void cv2typedVec(const cv::Mat& cvVec, TypedVec& typedVec,
				size_t size)
		{
			switch(cvVec.type())
			{
			case CV_32FC1:
				for(size_t i = 0; i < size; ++i)
				{
					typedVec[i] = cvVec.at<float>(i);
				}
				break;
			case CV_64FC1:
				for(size_t i = 0; i < size; ++i)
				{
					typedVec[i] = cvVec.at<double>(i);
				}
				break;
			default:
				std::stringstream message;
				message << "invalid cv matrix type: " << cvVec.type()
						<< std::flush;
				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR,
						message.str());
				break;
			}
		}

		// specialization to allow use of rvalue vector_range objects
		template<class TypedVec>
		inline void cv2range(const cv::Mat& cvVec, vector_range<TypedVec> vecRange,
				size_t size)
		{
			cv2typedVec(cvVec, vecRange, size);
		}

		template<class RawSpec>
		DataManagerMarkerFinderAruco<RawSpec>::
		DataManagerMarkerFinderAruco(float markerSize_)
			: ParentClass(markerSize_),
			  camParamsInitialized(false)
		{}

		template<class RawSpec>
		MarkerPtr DataManagerMarkerFinderAruco<RawSpec>::
		detectMarker(raw_ptr_t data)
		{
			if (not camParamsInitialized) setCamParams();
			MarkerPtr outMarkerP;

			boost::shared_ptr<RawSpec> rawData = SPTR_CAST<RawSpec>(data);
			const cv::Mat& image = rawData->img->mat();
			std::vector<aruco::Marker> arucoMarkers;

			mDetector.detect(image, arucoMarkers, camParams, this->markerSize);

			if (not arucoMarkers.empty() and camParams.isValid())
			{
				outMarkerP = boost::make_shared<Marker>();
				aruco::Marker& firstMarker = arucoMarkers[0];
				outMarkerP->id = firstMarker.id;
				cv2range(firstMarker.Tvec, outMarkerP->translation(), 3);
				jblas::vec3 rvec;
				cv2typedVec(firstMarker.Rvec, rvec, 3);
				outMarkerP->quaternion() = quaternion::v2q(rvec);
 			}
			return outMarkerP;
		}

		template<class RawSpec>
		void DataManagerMarkerFinderAruco<RawSpec>::
		setCamParams()
		{
			SensorImageParameters& rtslamParams = this->sensorPinhole().params;

			cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_32FC1);
			// | focalX   0    centerX |
			// |    0   focalY centerY |
			// |    0     0        1   |

			cameraMatrix.at<float>(0, 2) = rtslamParams.intrinsic[0]; // centerX
			cameraMatrix.at<float>(1, 2) = rtslamParams.intrinsic[1]; // centerY
			cameraMatrix.at<float>(0, 0) = rtslamParams.intrinsic[2]; // focalX
			cameraMatrix.at<float>(1, 1) = rtslamParams.intrinsic[3]; // focalY

			cv::Mat distCoeffs = cv::Mat::zeros(4,1,CV_32FC1);

			distCoeffs.at<float>(0) = rtslamParams.distortion[0];
			distCoeffs.at<float>(1) = rtslamParams.distortion[1];
			// rtslam has no tangential distortion
			distCoeffs.at<float>(2) = 0.0f;
			distCoeffs.at<float>(3) = 0.0f;

			camParams.setParams(cameraMatrix, distCoeffs,
					cv::Size(rtslamParams.width, rtslamParams.height));

			camParamsInitialized = true;
		}


	} // namespace ::rtslam
} // namespace jafar::

#endif // HAVE_ARUCO
