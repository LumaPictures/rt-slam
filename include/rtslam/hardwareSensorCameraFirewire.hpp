/**
 * \file hardwareSensorCameraFirewire.hpp
 *
 * Header file for getting data from a firewire camera
 *
 * \date 18/06/2010
 * \author croussil
 *
 * \ingroup rtslam
 */

#ifndef HARDWARE_SENSOR_CAMERA_FIREWIRE_HPP_
#define HARDWARE_SENSOR_CAMERA_FIREWIRE_HPP_

#include <jafarConfig.h>

#ifdef HAVE_VIAM
#include <viam/viamlib.h>
#endif

#include "rtslam/hardwareSensorCamera.hpp"
#include "rtslam/rawImage.hpp"


namespace jafar {
namespace rtslam {
namespace hardware {

/**
This class allows to get images from firewire with non blocking procedure,
using triple-buffering.
*/
class HardwareSensorCameraFirewire: public HardwareSensorCamera
{
	private:
#ifdef HAVE_VIAM
		viam_bank_t bank;
		viam_handle_t handle;
		
		struct ViamFormat
		{
			viam_hwmode_t hwmode;
			std::list<viam_filterformat_t> filters; // filters to get gray8 in the end
		};
#endif

		double realFreq;
		double last_timestamp;
		
		int mode;
		
		void preloadTask(void);
	
#ifdef HAVE_VIAM
		cv::Size viamSize_to_size(viam_hwsize_t hwsize);
		double viamFreq_to_freq(viam_hwfps_t hwfreq);
		
		viam_hwsize_t size_to_viamSize(cv::Size size);
		viam_hwfps_t freq_to_viamFreq(double freq);
		void format_to_viamFormat(cv::Size size, int format, viam_hwcrop_t crop, double freq, int trigger, ViamFormat & viam_format);
		viam_hwtrigger_t trigger_to_viamTrigger(int trigger);
#endif
	
		void init(int mode, std::string dump_path, cv::Size imgSize);
#ifdef HAVE_VIAM
		void init(const std::string &camera_id, ViamFormat &viam_format, double shutter, int mode, std::string dump_path);
#endif
	public:
		
#ifdef HAVE_VIAM
		/**
		@param camera_id the Firewire camera id (0x....)
		@param format
			0: GRAY8
			10: RGB24
			20: YUV411
			21: YUV422_UYVY
			22: YUV422_YUYV
			23: YUV422_YYUV
			24: YVU422_VYUY
			25: YVU422_YVYU
			26: YUV444
			30: BAYER_BGGR
			31: BAYER_GRBG
			32: BAYER_RGGB
			33: BAYER_GBRG
		@param mode 0 = normal, 1 = dump used images, 2 = from dumped images
		@param dump_path the path where the images are saved/read... Use a ram disk !!!
		*/
		HardwareSensorCameraFirewire(kernel::VariableCondition<int> *condition, int bufferSize, const std::string &camera_id, cv::Size size, int format, viam_hwcrop_t crop, double freq, int trigger, double shutter, int mode = 0, std::string dump_path = ".");
#endif
		/**
		Same as before but assumes that mode=2, and doesn't need a camera
		*/
		HardwareSensorCameraFirewire(kernel::VariableCondition<int> *condition, cv::Size imgSize, std::string dump_path = ".");
		
		~HardwareSensorCameraFirewire();

		virtual void start();
		virtual double getLastTimestamp() { boost::unique_lock<boost::mutex> l(mutex_data); return last_timestamp; }
		double getFreq() { return realFreq; }
};

typedef boost::shared_ptr<HardwareSensorCameraFirewire> hardware_sensor_firewire_ptr_t;

}}}

#endif
