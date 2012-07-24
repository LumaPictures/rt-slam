/**
 * \file HardwareSensorCamera.cpp
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
#include "rtslam/hardwareSensorCamera.hpp"


#include <image/Image.hpp>

namespace jafar {
namespace rtslam {
namespace hardware {


	void HardwareSensorCamera::preloadTaskOffline(void)
	{ JFR_GLOBAL_TRY
		int ndigit = 0;

		while(!stopping)
		{
			// acquire the image
			boost::unique_lock<boost::mutex> l(mutex_data);
			while (isFull(true)) cond_offline_freed.wait(l);
			l.unlock();
			int buff_write = getWritePos();
			while (!stopping)
			{
				// FIXME manage multisensors : put sensor id in filename
				std::ostringstream oss;
				for (int i = 3; i <= 7; ++i)
				{
					if (!found_first) ndigit = i;
					oss.str(""); oss << dump_path << "/image_" << std::setw(ndigit) << std::setfill('0') << index_load+first_index;
					if (found_first != 2 && bufferSpecPtr[buff_write]->img->load(oss.str() + std::string(".pgm"), 0) && found_first == 0) { found_first = 1; std::cout << "First image " << oss.str() << ".pgm" << std::endl; }
					if (found_first != 1 && bufferSpecPtr[buff_write]->img->load(oss.str() + std::string(".png"), 0) && found_first == 0) { found_first = 2; std::cout << "First image " << oss.str() << ".png" << std::endl; }
					if (found_first) break;
				}
				if (!found_first) { first_index++; continue; }

				if (bufferSpecPtr[buff_write]->img->data() == NULL)
				{
					boost::unique_lock<boost::mutex> l(mutex_data);
					no_more_data = true;
					//std::cout << "No more images to read." << std::endl;
					break;
				}
				std::fstream f((oss.str() + std::string(".time")).c_str(), std::ios_base::in);
				f >> bufferSpecPtr[buff_write]->timestamp; f.close();
				index_load++;
				break;
			}
			if (no_more_data) break;
			incWritePos();
			if (condition) condition->setAndNotify(1);
		}
		JFR_GLOBAL_CATCH
	}


	class LoggableImage: public kernel::Loggable
	{
	 private:
		std::string dump_path;
		rawimage_ptr_t image;
		int index;

	 public:
		LoggableImage(std::string const & dump_path, rawimage_ptr_t const & image, int index):
			dump_path(dump_path), image(image), index(index) {}
		virtual void log()
		{
			std::ostringstream oss; oss << dump_path << "/image_" << std::setw(7) << std::setfill('0') << index;
			image->img->save(oss.str() + std::string(".pgm"));
			std::fstream f; f.open((oss.str() + std::string(".time")).c_str(), std::ios_base::out);
			f << std::setprecision(20) << image->timestamp << std::endl; f.close();
		}
	};


	void HardwareSensorCamera::savePushTask(void)
	{ JFR_GLOBAL_TRY
		int last_processed_index = index();
		
		// clean previously existing files
		#if 0
		// TODO test
		boost::filesystem::path bdump_path(dump_path);
		if (!exists(bdump_path) || !is_directory(bdump_path)) create_directory(bdump_path);
		
		boost::regex pattern1("*.pgm");
		boost::regex pattern2("*.time");
		for (boost::filesystem::recursive_directory_iterator it(bdump_path), end; it != end; ++it)
		{
			std::string name = it->path().leaf();
			if (boost::regex_match(name, pattern1) || boost::regex_match(name, pattern2))
				remove(it->path());
		}
		//remove(bdump_path / "*.pgm"); // FIXME possible ?
		#else
		std::ostringstream oss; oss << "mkdir -p " << dump_path << " ; rm -f " << dump_path << "/*.pgm ; rm -f " << dump_path << "/*.time" << std::endl;
		int r = system(oss.str().c_str());
		if (!r) {} // don't care
		#endif
		
		int save_index = 0;

		while (!stopping)
		{
			index.wait(boost::lambda::_1 != last_processed_index);
			// push image to file for saving
			rawimage_ptr_t img = rawimage_ptr_t(static_cast<RawImage*>(bufferSpecPtr[last_sent_pos]->clone()));
			loggerTask->push(new LoggableImage(dump_path, img, save_index));

			last_processed_index = index();
			++save_index;
		}
		JFR_GLOBAL_CATCH
	}

	
	void HardwareSensorCamera::init(std::string dump_path, cv::Size imgSize)
	{
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
	}


	void HardwareSensorCamera::start()
	{
		if (started) { std::cout << "Warning: This HardwareSensorCameraFirewire has already been started" << std::endl; return; }

		// start save tasks
		// the save push task is here to avoid blocking during image clone, and to automatically detect images that are used
		if (mode == 1)
			savePushTask_thread = new boost::thread(boost::bind(&HardwareSensorCamera::savePushTask,this));

		// start acquire task
		last_timestamp = kernel::Clock::getTime();
		if (mode == 2)
			preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorCamera::preloadTaskOffline,this));
		else
			preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorCamera::preloadTask,this));

		started = true;
	}

	void HardwareSensorCamera::stop()
	{
		if (!started) return;
		stopping = true;
		preloadTask_thread->interrupt();
		preloadTask_thread->join();
		if (mode == 1)
		{
			savePushTask_thread->interrupt();
			savePushTask_thread->join();
		}
	}

	
	HardwareSensorCamera::HardwareSensorCamera(kernel::VariableCondition<int> *condition, cv::Size imgSize, std::string dump_path):
		HardwareSensorExteroAbstract(condition, 3)
	{
		init(dump_path, imgSize);
	}

	HardwareSensorCamera::HardwareSensorCamera(kernel::VariableCondition<int> *condition, int bufferSize, kernel::LoggerTask *loggerTask):
		HardwareSensorExteroAbstract(condition, bufferSize), loggerTask(loggerTask)
	{}

	

}}}

