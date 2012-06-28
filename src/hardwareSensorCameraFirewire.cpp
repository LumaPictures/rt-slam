/**
 * \file hardwareSensorCameraFirewire.cpp
 * \date 18/06/2010
 * \author croussil
 * \ingroup rtslam
 */

#include <algorithm>
#include <sstream>
#include <fstream>

#if 0
// creates conflict with boost sandbox with boost 1.42 in debug
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#endif

#include "kernel/timingTools.hpp"
#include "rtslam/hardwareSensorCameraFirewire.hpp"

#ifdef HAVE_VIAM
#include <viam/viamcv.h>
#endif

#include <image/Image.hpp>

namespace jafar {
namespace rtslam {
namespace hardware {

#ifdef HAVE_VIAM
	cv::Size HardwareSensorCameraFirewire::viamSize_to_size(viam_hwsize_t hwsize)
	{
		switch (hwsize)
		{
			case VIAM_HWSZ_160x120: return cv::Size(160,120);
			case VIAM_HWSZ_320x240: return cv::Size(320,240);
			case VIAM_HWSZ_512x384: return cv::Size(512,384);
			case VIAM_HWSZ_640x480: return cv::Size(640,480);
			case VIAM_HWSZ_800x600: return cv::Size(800,600);
			case VIAM_HWSZ_1024x768: return cv::Size(1024,768);
			case VIAM_HWSZ_1280x960: return cv::Size(1280,960);
			case VIAM_HWSZ_1600x1200: return cv::Size(1600,1200);
			default: return cv::Size(0,0);
		}
	}
	
	double HardwareSensorCameraFirewire::viamFreq_to_freq(viam_hwfps_t hwfreq)
	{
		switch (hwfreq)
		{
			case VIAM_HWFPS_FREE: return 0.0;
			case VIAM_HWFPS_1_875: return 1.875;
			case VIAM_HWFPS_3_75: return 3.75;
			case VIAM_HWFPS_7_5: return 7.5;
			case VIAM_HWFPS_15: return 15.;
			case VIAM_HWFPS_30: return 30.;
			case VIAM_HWFPS_60: return 60.;
			case VIAM_HWFPS_120: return 120.;
			case VIAM_HWFPS_240: return 240;;
			default: return 0.;
		}
	}
	
	viam_hwsize_t HardwareSensorCameraFirewire::size_to_viamSize(cv::Size size)
	{
		viam_hwsize_t hwsize;
		switch (size.width)
		{
			case 160: if (size.height == 120) hwsize = VIAM_HWSZ_160x120; else hwsize = VIAM_HWSZ_INVALID; break;
			case 320: if (size.height == 240) hwsize = VIAM_HWSZ_320x240; else hwsize = VIAM_HWSZ_INVALID; break;
			case 512: if (size.height == 384) hwsize = VIAM_HWSZ_512x384; else hwsize = VIAM_HWSZ_INVALID; break;
			case 640: if (size.height == 480) hwsize = VIAM_HWSZ_640x480; else hwsize = VIAM_HWSZ_INVALID; break;
			case 800: if (size.height == 600) hwsize = VIAM_HWSZ_800x600; else hwsize = VIAM_HWSZ_INVALID; break;
			case 1024: if (size.height == 768) hwsize = VIAM_HWSZ_1024x768; else hwsize = VIAM_HWSZ_INVALID; break;
			case 1280: if (size.height == 960) hwsize = VIAM_HWSZ_1280x960; else hwsize = VIAM_HWSZ_INVALID; break;
			case 1600: if (size.height == 1200) hwsize = VIAM_HWSZ_1600x1200; else hwsize = VIAM_HWSZ_INVALID; break;
			default: hwsize = VIAM_HWSZ_INVALID;
		}
		return hwsize;
	}
/*
	viam_hwfps_t HardwareSensorCameraFirewire::freq_to_viamFreq(double freq)
	{
		viam_hwfps_t hwfreq;
		if (freq < (1.875+3.75)/2) hwfreq = VIAM_HWFPS_1_875; else
		if (freq < (3.75+7.5)/2) hwfreq = VIAM_HWFPS_3_75; else
		if (freq < (7.5+15)/2) hwfreq = VIAM_HWFPS_7_5; else
		if (freq < (15+30)/2) hwfreq = VIAM_HWFPS_15; else
		if (freq < (30+60)/2) hwfreq = VIAM_HWFPS_30; else
		if (freq < (60+120)/2) hwfreq = VIAM_HWFPS_60; else
		if (freq < (120+240)/2) hwfreq = VIAM_HWFPS_120; else
			hwfreq = VIAM_HWFPS_240;
		return hwfreq;
	}
*/
	viam_hwfps_t HardwareSensorCameraFirewire::freq_to_viamFreq(double freq)
	{
		freq -= 0.1;
		viam_hwfps_t hwfreq;
		if (freq <= 0.0) hwfreq = VIAM_HWFPS_FREE; else
		if (freq <= 1.875) hwfreq = VIAM_HWFPS_1_875; else
		if (freq <= 3.75) hwfreq = VIAM_HWFPS_3_75; else
		if (freq <= 7.5) hwfreq = VIAM_HWFPS_7_5; else
		if (freq <= 15) hwfreq = VIAM_HWFPS_15; else
		if (freq <= 30) hwfreq = VIAM_HWFPS_30; else
		if (freq <= 60) hwfreq = VIAM_HWFPS_60; else
		if (freq <= 120) hwfreq = VIAM_HWFPS_120; else
			hwfreq = VIAM_HWFPS_240;
		return hwfreq;
	}

	viam_hwtrigger_t HardwareSensorCameraFirewire::trigger_to_viamTrigger(int trigger)
	{
		viam_hwtrigger_t hwtrigger;
		switch (trigger)
		{
			case 1: hwtrigger = VIAM_HWTRIGGER_MODE1_HIGH; break;
			case 2: hwtrigger = VIAM_HWTRIGGER_MODE0_HIGH; break;
			case 3: std::cout << "error: trigger mode 14 not implemented in viam, switch to mode 0" << std::endl;
			        hwtrigger = VIAM_HWTRIGGER_MODE0_HIGH; break;
			default:
			case 0: hwtrigger = VIAM_HWTRIGGER_INTERNAL;
		}
		return hwtrigger;
	}

	void HardwareSensorCameraFirewire::format_to_viamFormat(cv::Size size, int format, viam_hwcrop_t crop, double freq, int trigger, ViamFormat & viam_format)
	{
		// fill filters and format
		viam_hwformat_t hwformat = VIAM_HWFMT_INVALID;
		switch (format / 10)
		{
			case 0: // gray
				hwformat = VIAM_HWFMT_MONO8;
				if (format > 0) std::cerr << "Bad image format " << format << std::endl;
				break;
			case 1: // RGB
				hwformat = VIAM_HWFMT_RGB888;
				if (format > 10) std::cerr << "Bad image format " << format << std::endl;
				break;
			case 2: // YUV
				switch (format % 10)
				{
					case 0: hwformat = VIAM_HWFMT_YUV411; viam_format.filters.push_back(VIAM_FILTER_YUV411_TO_BGR); break;
					case 1: hwformat = VIAM_HWFMT_YUV422_UYVY; viam_format.filters.push_back(VIAM_FILTER_YUV422_UYVY_TO_BGR); break;
					case 2: hwformat = VIAM_HWFMT_YUV422_YUYV; viam_format.filters.push_back(VIAM_FILTER_YUV422_YUYV_TO_BGR); break;
					case 3: hwformat = VIAM_HWFMT_YUV422_YYUV; viam_format.filters.push_back(VIAM_FILTER_YUV422_YYUV_TO_BGR); break;
					case 4: hwformat = VIAM_HWFMT_YVU422_VYUY; viam_format.filters.push_back(VIAM_FILTER_YVU422_VYUY_TO_BGR); break;
					case 5: hwformat = VIAM_HWFMT_YVU422_YVYU; viam_format.filters.push_back(VIAM_FILTER_YVU422_YVYU_TO_BGR); break;
					case 6: hwformat = VIAM_HWFMT_YUV444; viam_format.filters.push_back(VIAM_FILTER_YUV444_TO_BGR); break;
					default: std::cerr << "Bad image format " << format << std::endl; break;
				}
				break;
			case 3: // bayer
				hwformat = VIAM_HWFMT_MONO8;
				switch (format % 10)
				{
					case 0: viam_format.filters.push_back(VIAM_FILTER_BAYER_NN_BGGR); break;
					case 1: viam_format.filters.push_back(VIAM_FILTER_BAYER_NN_GRBG); break;
					case 2: viam_format.filters.push_back(VIAM_FILTER_BAYER_NN_RGGB); break;
					case 3: viam_format.filters.push_back(VIAM_FILTER_BAYER_NN_GBRG); break;
					default: std::cerr << "Bad image format " << format << std::endl; break;
				}
				break;
			default: std::cerr << "Bad image format " << format << std::endl; break;
		}

		if (format > 10)
			viam_format.filters.push_back(VIAM_FILTER_BGR_TO_GRAY);
		else if (format == 10)
			viam_format.filters.push_back(VIAM_FILTER_RGB_TO_GRAY);

		// fill format
		viam_format.hwmode.size = size_to_viamSize(size);
		viam_format.hwmode.format = hwformat;
		viam_format.hwmode.crop = crop;
		viam_format.hwmode.fps = freq_to_viamFreq(freq);
		viam_format.hwmode.trigger = trigger_to_viamTrigger(trigger);
	}

#endif

	void HardwareSensorCameraFirewire::preloadTask(void)
	{ try {
		struct timeval ts, *pts = &ts;
		int r;
		//bool emptied_buffers = false;
		//double date = 0.;

		while(true)
		{
			// acquire the image
#ifdef HAVE_VIAM
			int buff_write = getWritePos();
			//if (!emptied_buffers) date = kernel::Clock::getTime();
			r = viam_oneshot(handle, bank, &(bufferImage[buff_write]), &pts, 1);
			if (r) continue;
			//if (!emptied_buffers) { date = kernel::Clock::getTime()-date; if (date < 0.004) continue; else emptied_buffers = true; }
			bufferSpecPtr[buff_write]->arrival = kernel::Clock::getTime();
			bufferSpecPtr[buff_write]->timestamp = ts.tv_sec + ts.tv_usec*1e-6;
			boost::unique_lock<boost::mutex> l(mutex_data);
			arrival_delay = bufferSpecPtr[buff_write]->arrival - bufferSpecPtr[buff_write]->timestamp;
			last_timestamp = bufferSpecPtr[buff_write]->timestamp;
			incWritePos(true);
			l.unlock();
#endif
			if (condition) condition->setAndNotify(1);
		}
	} catch (kernel::Exception &e) { std::cout << e.what(); throw e; } }

	
	void HardwareSensorCameraFirewire::init(int mode, std::string dump_path, cv::Size imgSize)
	{
		this->mode = mode;
		this->dump_path = dump_path;

		// configure data
		bufferImage.resize(bufferSize);
		bufferSpecPtr.resize(bufferSize);
		for(int i = 0; i < bufferSize; ++i)
		{
			bufferImage[i] = cvCreateImage(imgSize, 8, 1);
			buffer(i).reset(new RawImage());
			bufferSpecPtr[i] = SPTR_CAST<RawImage>(buffer(i));
			bufferSpecPtr[i]->setJafarImage(jafarImage_ptr_t(new image::Image(bufferImage[i])));
		}
		
		found_first = 0;
		first_index = 0;
		index_load = 0;

		// start save tasks
		if (mode == 1)
		{
			saveTask_thread = new boost::thread(boost::bind(&HardwareSensorCameraFirewire::saveTask,this));
			savePushTask_thread = new boost::thread(boost::bind(&HardwareSensorCameraFirewire::savePushTask,this));
		}
		
	}
	
	void HardwareSensorCameraFirewire::start()
	{
		// start acquire task
		if (started) { std::cout << "Warning: This HardwareSensorCameraFirewire has already been started" << std::endl; return; }
		started = true;
		last_timestamp = kernel::Clock::getTime();
		if (mode == 2)
			preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorCameraFirewire::preloadTaskOffline,this));
		else
			preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorCameraFirewire::preloadTask,this));
	}
		
	
	HardwareSensorCameraFirewire::HardwareSensorCameraFirewire(kernel::VariableCondition<int> *condition, cv::Size imgSize, std::string dump_path):
		HardwareSensorCamera(condition, imgSize, dump_path)
	{}
	

#ifdef HAVE_VIAM
	void HardwareSensorCameraFirewire::init(const std::string &camera_id, ViamFormat &viam_format, double shutter, int mode, std::string dump_path)
	{
		// configure camera
		if (mode == 0 || mode == 1)
		{
			int r;
			handle = viam_init();
			viam_camera_t camera = viam_camera_create(handle, camera_id.c_str(), "camera1");
			if (!camera) std::cerr << "viam_camera_create failed" << std::endl;
			bank = viam_bank_create(handle,"bank1");
			if (!bank) std::cerr << "viam_bank_create failed" << std::endl;
			r = viam_bank_cameraadd(handle,bank,camera,"image1");
			if (r) std::cerr << "viam_bank_cameraadd failed with error " << r << std::endl;
			r = viam_camera_sethwmode(handle, camera, &(viam_format.hwmode));
			if (r) std::cerr << "viam_camera_sethwmode failed with error " << r << std::endl;
			r = viam_hardware_load(handle,"dc1394");
			if (r) std::cerr << "viam_hardware_load failed with error " << r << std::endl;

			r = viam_hardware_attach(handle);
			if (r) std::cerr << "viam_hardware_attach failed with error " << r << std::endl;
			r = viam_bank_configure(handle, bank);
			if (r) std::cerr << "viam_bank_configure failed with error " << r << std::endl;

			viam_image_ref image = viam_image_getbyname(handle, "image1");

			int i = 0;
			for(std::list<viam_filterformat_t>::iterator it = viam_format.filters.begin(); it != viam_format.filters.end(); ++it)
			{
				std::ostringstream oss; oss << i++;
				viam_filter_t filter = viam_filter_format_create(handle, oss.str().c_str(), VIAM_FILTER_FORMAT, *it);
				if (!filter) std::cerr << "viam_filter_format_create failed" << std::endl;
				viam_filter_push(handle, image, filter, VIAM_FILTER_SOFTWARE, VIAM_FILTER_MANUAL);
				if (r) std::cerr << "viam_filter_push failed with error " << r << std::endl;
			}

			if (viam_format.hwmode.trigger != VIAM_HWTRIGGER_MODE1_HIGH)
			{ 
				viam_filter_t shutter_filter = viam_filter_luminance_create(handle, "shutter", VIAM_FILTER_SHUTTER, VIAM_VALUE_ABSOLUTE, shutter);
				if (!shutter_filter) std::cerr << "viam_filter_luminance_create failed" << std::endl;
				if (shutter >= 1e-6)
					r = viam_filter_push(handle, image, shutter_filter, VIAM_FILTER_HARDWARE, VIAM_FILTER_MANUAL);
				else
					r = viam_filter_push(handle, image, shutter_filter, VIAM_FILTER_HARDWARE, VIAM_FILTER_HARDWARE_AUTO);
				if (r) std::cerr << "viam_filter_push failed with error " << r << std::endl;
			}
			
			r = viam_datatransmit(handle, bank, VIAM_ON);
			if (r) std::cerr << "viam_datatransmit failed with error " << r << std::endl;
		}

		init(mode, dump_path, viamSize_to_size(viam_format.hwmode.size));
	}

	HardwareSensorCameraFirewire::HardwareSensorCameraFirewire(kernel::VariableCondition<int> *condition, int bufferSize, const std::string &camera_id, cv::Size size, int format, viam_hwcrop_t crop, double freq, int trigger, double shutter, int mode, std::string dump_path):
		HardwareSensorCamera(condition, bufferSize)
	{
		ViamFormat viam_format;
		format_to_viamFormat(size, format, crop, freq, trigger, viam_format);
		realFreq = viamFreq_to_freq(viam_format.hwmode.fps);
		std::cout << "Camera set to freq " << realFreq << " Hz (external trigger " << trigger << ")" << std::endl;
		init(camera_id, viam_format, shutter, mode, dump_path);
	}
#endif


	HardwareSensorCameraFirewire::~HardwareSensorCameraFirewire()
	{
#ifdef HAVE_VIAM
		if (mode == 0 || mode == 1)
			viam_release(handle);
		saveTask_cond.wait(boost::lambda::_1 == 0);
#endif
	}



}}}

