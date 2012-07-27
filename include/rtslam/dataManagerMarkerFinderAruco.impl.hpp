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

		// FIXME: don't hardcode marker size / cam param file
		template<class RawSpec, class SensorSpec>
		DataManagerMarkerFinderAruco<RawSpec, SensorSpec>::
		DataManagerMarkerFinderAruco()
			: markerSize(.207)
		{
			camParams.readFromXMLFile("/mnt/ssd/AR/aruco/build/Linux-x86_64/debug/Flea3_intrinsics.yml");
			//camParams.resize(TheInputImage.size());
		}

		template<class RawSpec, class SensorSpec>
		MarkerPtr DataManagerMarkerFinderAruco<RawSpec, SensorSpec>::
		detectMarker(raw_ptr_t data)
		{
			MarkerPtr outMarkerP;

			boost::shared_ptr<RawSpec> rawData = SPTR_CAST<RawSpec>(data);
			const cv::Mat& image = rawData->img->mat();
			std::vector<aruco::Marker> arucoMarkers;

			mDetector.detect(image, arucoMarkers, camParams, markerSize);

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


	} // namespace ::rtslam
} // namespace jafar::

#endif // HAVE_ARUCO
