/**
 * \file hardwareSensorMti.cpp
 * \date 18/06/2010
 * \author croussil
 * \ingroup rtslam
 */

#include "rtslam/hardwareSensorMti.hpp"

#include <sys/time.h>
#include <boost/bind.hpp>

#include "kernel/jafarMacro.hpp"
#include "kernel/timingTools.hpp"
#include "jmath/misc.hpp"
#include "jmath/indirectArray.hpp"

#include "rtslam/rtslamException.hpp"

namespace jafar {
namespace rtslam {
namespace hardware {

	void HardwareSensorMti::preloadTask(void)
	{ JFR_GLOBAL_TRY
#ifdef HAVE_MTI
		INERTIAL_DATA data;
#endif
		//bool emptied_buffers = false; // done in driver
		//double date = 0.;
		std::fstream f;
		if (mode == 1 || mode == 2)
		{
			std::ostringstream oss; oss << dump_path << "/MTI.log";
			f.open(oss.str().c_str(), (mode == 1 ? std::ios_base::out : std::ios_base::in));
		}
		
		while (!stopping)
		{
			if (mode == 2)
			{
				f >> reading.data;
				boost::unique_lock<boost::mutex> l(mutex_data);
				if (isFull(true)) cond_offline_full.notify_all();
				if (f.eof()) { no_more_data = true; cond_offline_full.notify_all(); f.close(); return; }
				while (!stopping && isFull(true)) cond_offline_freed.wait(l);
				if (stopping) break;

			} else
			{
#ifdef HAVE_MTI
				if (!mti->read(&data)) continue;
				if (isFull()) JFR_ERROR(RtslamException, RtslamException::BUFFER_OVERFLOW, "Data not read: Increase MTI buffer size !");

				reading.arrival = kernel::Clock::getTime();
				reading.data(0) = data.TIMESTAMP_FILTERED;
				reading.data(1) = data.ACC[0];
				reading.data(2) = data.ACC[1];
				reading.data(3) = data.ACC[2];
				reading.data(4) = data.GYR[0];
				reading.data(5) = data.GYR[1];
				reading.data(6) = data.GYR[2];
				reading.data(7) = data.MAG[0];
				reading.data(8) = data.MAG[1];
				reading.data(9) = data.MAG[2];
				arrival_delay = reading.arrival - reading.data(0);
#endif
			}
			int buff_write = getWritePos(true); // don't need to lock because we are the only writer
			buffer(buff_write).data = reading.data;
			buffer(buff_write).data(0) += timestamps_correction;
			incWritePos();
			if (condition) condition->setAndNotify(1);
			
			if (mode == 1)
				loggerTask->push(new LoggableProprio(f, reading.data));

		}
		
		if (mode == 1 || mode == 2)
			f.close();
		
		JFR_GLOBAL_CATCH
	}

	HardwareSensorMti::HardwareSensorMti(kernel::VariableCondition<int> *condition, std::string device, double trigger_mode,
		double trigger_freq, double trigger_shutter, int bufferSize_, int mode, std::string dump_path, kernel::LoggerTask *loggerTask):
		HardwareSensorProprioAbstract(condition, bufferSize_, ctNone),
#ifdef HAVE_MTI
		mti(NULL),
#endif
		/*tightly_synchronized(false), */ mode(mode), dump_path(dump_path), loggerTask(loggerTask)
	{
		if (mode == 1 && !loggerTask) JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "HardwareSensorMti: you must provide a loggerTask if you want to dump data.");
		addQuantity(qAcc);
		addQuantity(qAngVel);
		addQuantity(qMag);
		initData();


		if (mode != 2)
		{
#ifdef HAVE_MTI
			mti = new MTI(device.c_str(), MTI_OPMODE_CALIBRATED, MTI_OPFORMAT_MAT, MTI_SYNCOUTMODE_DISABLED);

			if (not mti->is_connected()) JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "HardwareSensorMti: failed to connect to mti.");

			INERTIAL_CONFIG config;
			// default syncout pin modes and settings
			config.syncOutMode          = MTI_SYNCOUTMODE_PULSE;
			config.syncOutPulsePolarity = MTI_SYNCOUTPULSE_POS;
			// number of acquisitions to skip before syncOut pin actuates
			
			if (trigger_mode != 0)
			{
				const double period = 10e-3; // 10ms = 100Hz
				config.syncOutSkipFactor = std::floor(1. / trigger_freq / period + 0.001) - 1; // mark all acquisitions
				realFreq = 1. / (period * (config.syncOutSkipFactor + 1) );
				std::cout << "MTI trigger set to freq " << realFreq << " Hz" << std::endl;
				// number of ns to offset pin action from sensor sampling
				config.syncOutOffset = 0; // no offset
				// number of ns to define pulse width
				if (trigger_shutter < 1e-6 || trigger_mode != 1) trigger_shutter = 0.5e-3;
//				config.syncOutPulseWidth = trigger_shutter/1e-9; // new driver
				config.syncOutPulseWidth = trigger_shutter/33.9e-9; // old driver
	
				// Set SyncOut settings
				if (!mti->set_syncOut(config.syncOutMode, config.syncOutPulsePolarity,
					config.syncOutSkipFactor, config.syncOutOffset,
					config.syncOutPulseWidth))
					std::cout << "mti.set_syncOut failed" << std::endl;
			}
//if (!mti.set_outputSkipFactor(49))
//std::cout << "mti.set_outputFactor failed" << std::endl;
#endif
		} else
		{
			realFreq = trigger_freq;
		}
		
	}
	
	
	HardwareSensorMti::~HardwareSensorMti()
	{
#ifdef HAVE_MTI
		delete mti;
#endif
	}

	void HardwareSensorMti::start()
	{
		if (started) { std::cout << "Warning: This HardwareSensorMti has already been started" << std::endl; return; }
		started = true;
		last_timestamp = kernel::Clock::getTime();

		// start acquire task
		preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorMti::preloadTask,this));
		if (mode == 2)
		{ // wait that log has been read before first frame
			boost::unique_lock<boost::mutex> l(mutex_data);
			cond_offline_full.wait(l);
		}
		std::cout << " done." << std::endl;
	}
	
	void HardwareSensorMti::stop()
	{
		if (!started) return;
		stopping = true;
		cond_offline_freed.notify_all();
		preloadTask_thread->join();
	}

	void HardwareSensorMti::setSyncConfig(double timestamps_correction/*, bool tightly_synchronized, double tight_offset*/)
	{
		this->timestamps_correction = timestamps_correction;
		//this->tightly_synchronized = tightly_synchronized;
		//this->tight_offset = tight_offset;
	}



}}}

