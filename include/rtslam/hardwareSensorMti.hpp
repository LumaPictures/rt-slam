/**
 * \file hardwareSensorMti.hpp
 *
 * Header file for hardware robots
 *
 * \date 18/06/2010
 * \author croussil
 *
 * \ingroup rtslam
 */

#ifndef HARDWARE_SENSOR_MTI_HPP_
#define HARDWARE_SENSOR_MTI_HPP_

#include "jafarConfig.h"

#ifdef HAVE_MTI
#include <MTI-clients/MTI.h>
#endif

#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "jmath/jblas.hpp"
#include "jmath/indirectArray.hpp"

#include "rtslam/hardwareSensorAbstract.hpp"



namespace jafar {
namespace rtslam {
namespace hardware {

	class HardwareSensorMti: public HardwareSensorProprioAbstract
	{
		private:
#ifdef HAVE_MTI
			MTI *mti;
#endif
			
			int mode;
			std::string dump_path;
			double realFreq;
			double last_timestamp;
			kernel::LoggerTask *loggerTask;

			boost::thread *preloadTask_thread;
			void preloadTask(void);
		
		public:
			
			HardwareSensorMti(kernel::VariableCondition<int> *condition, std::string device, double trigger_mode,
				double trigger_freq, double trigger_shutter, int bufferSize_, int mode = 0, std::string dump_path = ".", kernel::LoggerTask *loggerTask = NULL);
			virtual ~HardwareSensorMti();
			virtual void start();
			virtual void stop();
			virtual double getLastTimestamp() { boost::unique_lock<boost::mutex> l(mutex_data); return last_timestamp; }
			void setSyncConfig(double timestamps_correction = 0.0/*, bool tightly_synchronized = false, double tight_offset*/);
			
			/**
			 * @return data with 10 columns: time, accelero (3), gyro (3), magneto (3)
			 */
			jblas::ind_array instantValues() { return jmath::ublasExtra::ia_set(1,10); }
			jblas::ind_array incrementValues() { return jmath::ublasExtra::ia_set(1,1); }

			double getFreq() { return realFreq; } // trigger freq
	};

}}}

#endif

