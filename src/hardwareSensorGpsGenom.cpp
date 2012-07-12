/**
 * \file hardwareSensorGpsGenom.cpp
 *
 * File for getting data from the genom gps module
 *
 * \date 16/03/2011
 * \author croussil
 *
 * \ingroup rtslam
 */

#include "kernel/timingTools.hpp"
#include "rtslam/hardwareSensorGpsGenom.hpp"

#ifdef HAVE_POSTERLIB
#include "h2timeLib.h"
#endif


namespace jafar {
namespace rtslam {
namespace hardware {


	void HardwareSensorGpsGenom::preloadTask(void)
	{ JFR_GLOBAL_TRY
		char data[256];
#ifdef HAVE_POSTERLIB
		H2TIME h2timestamp;
#endif
		unsigned long prev_ntick = 0;
		double *date, prev_date = 0.0;
		double *pos; float *var;
		
		std::fstream f;
		if (mode == 1 || mode == 2)
		{
			std::ostringstream oss; oss << dump_path << "/GPS.log";
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
				while (isFull(true)) cond_offline_freed.wait(l);
				
			} else
			{
#ifdef HAVE_POSTERLIB
				while (!stopping) // wait for new data
				{
					usleep(1000);
					if (posterIoctl(posterId, FIO_GETDATE, &h2timestamp) != ERROR)
					{
						if (h2timestamp.ntick != prev_ntick)
						{
							prev_ntick = h2timestamp.ntick;
							if (posterRead(posterId, 0, data, 256) != ERROR)
							{
								date = (double*)(data+80+44);
								if (*date != prev_date)
								{
									prev_date = *date;
									break;
								}
							}
						}
					}
				}
#endif
				if (isFull()) JFR_ERROR(RtslamException, RtslamException::BUFFER_OVERFLOW, "Data not read: Increase GPS buffer size !");
				reading.arrival = kernel::Clock::getTime();
				pos = (double*)(data+16);
				var = (float*)(data+48);
				reading.data(0) = *date;
				reading.data(1) = pos[1]; // swap
				reading.data(2) = pos[0];
				reading.data(3) = pos[2];
				reading.data(4) = var[1]; // swap
				reading.data(5) = var[0];
				reading.data(6) = var[2];
				//std::cout << "GPS poster : " << std::setprecision(15) << reading.data << std::endl;
				arrival_delay = reading.arrival - reading.data(0);
			}
			
			int buff_write = getWritePos(true); // don't need to lock because we are the only writer
			buffer(buff_write).data = reading.data;
			buffer(buff_write).data(0) += timestamps_correction;
			last_timestamp = reading.data(0);
			incWritePos();
			if (condition) condition->setAndNotify(1);
			
			if (mode == 1)
				loggerTask->push(new LoggableProprio(f, reading.data));

		}
		JFR_GLOBAL_CATCH
	}
	
	
	HardwareSensorGpsGenom::HardwareSensorGpsGenom(kernel::VariableCondition<int> *condition, unsigned bufferSize, const std::string machine, int mode, std::string dump_path, kernel::LoggerTask *loggerTask):
		HardwareSensorProprioAbstract(condition, bufferSize, ctVar), mode(mode), dump_path(dump_path), loggerTask(loggerTask)
	{
		if (mode == 1 && !loggerTask) JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "HardwareSensorGpsGenom: you must provide a loggerTask if you want to dump data.");
		addQuantity(qPos);
		//addQuantity(qAbsVel);
		initData();

		// configure
		if (mode == 0 || mode == 1)
		{
#ifdef HAVE_POSTERLIB
			char *backup_poster_path = getenv("POSTER_PATH");
			setenv("POSTER_PATH", machine.c_str(), 1);
			if (posterFind("GPSInfo", &this->posterId) == ERROR) {
				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "Poster GPSInfo could not be found");
			}
			if (backup_poster_path) setenv("POSTER_PATH", backup_poster_path, 1); else unsetenv("POSTER_PATH");
#else
			JFR_ERROR(RtslamException, RtslamException::MISSING_DEPENDENCY,
			          "You must install posterlib (pocolibs) if you want to use online GPS sensor");
#endif
		}
	}

	void HardwareSensorGpsGenom::start()
	{
		// start acquire task
		//preloadTask();
		if (started) { std::cout << "Warning: This HardwareSensorGpsGenom has already been started" << std::endl; return; }
		started = true;
		last_timestamp = kernel::Clock::getTime();
		preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorGpsGenom::preloadTask,this));
		
		if (mode == 2)
		{ // wait that log has been read before first frame
			boost::unique_lock<boost::mutex> l(mutex_data);
			cond_offline_full.wait(l);
		}
	}

	void HardwareSensorGpsGenom::stop()
	{
		if (!started) return;
		stopping = true;
		preloadTask_thread->join();
	}

	
}}}

