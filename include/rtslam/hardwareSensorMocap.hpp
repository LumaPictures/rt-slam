/**
 * \file hardwareSensorGpsGenom.hpp
 *
 * Header file for getting data from the genom gps module
 *
 * \date 16/03/2011
 * \author croussil
 *
 * \ingroup rtslam
 */

#ifndef HARDWARE_SENSOR_MOCAP_HPP_
#define HARDWARE_SENSOR_MOCAP_HPP_

#include <stdlib.h>
#include <unistd.h>

#include <jafarConfig.h>
#include "kernel/jafarMacro.hpp"
#include "rtslam/hardwareSensorAbstract.hpp"

namespace jafar {
namespace rtslam {
namespace hardware {


class HardwareSensorMocap: public HardwareSensorProprioAbstract
{
	private:
		boost::thread *preloadTask_thread;
		void preloadTask(void);
		
		int mode;
		std::string dump_path;
		double last_timestamp;
		
	public:
		HardwareSensorMocap(kernel::VariableCondition<int> *condition, unsigned bufferSize, int mode = 0, std::string dump_path = ".");
		
		virtual void start();
		virtual void stop();
		virtual double getLastTimestamp() { boost::unique_lock<boost::mutex> l(mutex_data); return last_timestamp; }
		
		jblas::ind_array instantValues() { return jmath::ublasExtra::ia_set(1,7); }
		jblas::ind_array incrementValues() { return jmath::ublasExtra::ia_set(1,1); }
};


}}}

#endif // HARDWARE_SENSOR_MOCAP_HPP_
