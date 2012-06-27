/**
 * \file robotAbstract.cpp
 * \date 08/03/2010
 * \author jsola
 * \ingroup rtslam
 */

#include "rtslam/robotAbstract.hpp"
#include "rtslam/sensorAbstract.hpp"
#include "rtslam/mapAbstract.hpp"

#include "rtslam/quatTools.hpp"
#include "jmath/angle.hpp"

#include <boost/shared_ptr.hpp>

namespace jafar {
	namespace rtslam {
		using namespace std;

		IdFactory RobotAbstract::robotIds = IdFactory();

		/*
		 * Operator << for class RobotAbstract.
		 * It shows different information of the robot.
		 */
		ostream& operator <<(ostream & s, RobotAbstract const & rob) {
			s << rob.categoryName() << " " << rob.id() << ": ";
			if (rob.name().size() > 0) s << rob.name() << ", ";
			s << "of type " << rob.typeName() << endl;
			s << ".state:  " << rob.state << endl;
//			s << ".pose :  " << rob.pose << endl;
			s << ".st.pert:   " << rob.Q << endl;
			s << ".sensor list : [";
			for (RobotAbstract::SensorList::const_iterator senIter = rob.sensorList().begin();
						senIter != rob.sensorList().end(); senIter++)
				s << " " << (*senIter)->id() << " "; // Print the address of the sensor.
			s << "]";
			return s;
		}


		/*
		 * Remote constructor from remote map and size of control vector.
		 */
		RobotAbstract::RobotAbstract(const map_ptr_t & _mapPtr, const size_t _size_state, const size_t _size_control, const size_t _size_pert) :
			MapObject(_mapPtr, _size_state),
			pose(state, jmath::ublasExtra::ia_set(0, 7)),
			control(_size_control),
			perturbation(_size_pert),
			XNEW_x(_size_state, _size_state),
			XNEW_pert(_size_state, _size_pert),
			Q(_size_state, _size_state),
			origin_sensors(3), origin_export(3), robot_pose(6),
			extrapol_up_to_date(false)
		{
			constantPerturbation = false;
			category = ROBOT;
			self_time = -1.;
			Q.clear();
			origin_sensors.clear();
			origin_export.clear();
			robot_pose.clear();
		}

		RobotAbstract::RobotAbstract(const simulation_t dummy, const map_ptr_t & _mapPtr, const size_t _size_state, const size_t _size_control, const size_t _size_pert) :
			MapObject(_mapPtr, _size_state, UNFILTERED),
			pose(state, jmath::ublasExtra::ia_set(0, 7)),
			control(_size_control),
			perturbation(_size_pert),
			XNEW_x(_size_state, _size_state),
			XNEW_pert(_size_state, _size_pert),
			Q(_size_state, _size_state),
			origin_sensors(3), origin_export(3), robot_pose(6),
			extrapol_up_to_date(false)
		{
			constantPerturbation = true;
			category = ROBOT;
			self_time = -1.;
			origin_sensors.clear();
			origin_export.clear();
			robot_pose.clear();
		}

		void RobotAbstract::setPoseDegStd(double x, double y, double z, double rollDeg,
		    double pitchDeg, double yawDeg, double xStd, double yStd, double zStd,
		    double rollDegStd, double pitchDegStd, double yawDegStd)
		{
			setPoseStd(x,y,z, jmath::degToRad(rollDeg), jmath::degToRad(pitchDeg), jmath::degToRad(yawDeg),
			           xStd, yStd, zStd, jmath::degToRad(rollDegStd), jmath::degToRad(pitchDegStd), jmath::degToRad(yawDegStd));
		}

		void RobotAbstract::setPoseStd(double x, double y, double z, double roll,
		    double pitch, double yaw, double xStd, double yStd, double zStd,
		    double rollStd, double pitchStd, double yawStd)
		{

			const double pos_[3] = { x, y, z };
			const double euler_[3] = { roll, pitch, yaw };

			// convert euler pose to quat pose
			ublas::subrange(pose.x(), 0, 3) = createVector<3> (pos_);

			vec3 euler = createVector<3> (euler_);
			ublas::subrange(pose.x(), 3, 7) = quaternion::e2q(euler);

			// convert euler uncertainty to quaternion uncertainty
			const double posStd_[3] = { xStd, yStd, zStd };
			const double eulerStd_[3] = { rollStd, pitchStd, yawStd };

			vec3 eulerStd = createVector<3> (eulerStd_);
			Gaussian E(3);	E.std(eulerStd);
			vec4 q;
			mat Q_e(4, 3);

			quaternion::e2q(euler, q, Q_e);

			// write pose
			subrange(pose.P(), 0,3, 0,3) = createSymMat<3>(posStd_);
			subrange(pose.P(), 3,7, 3,7) = prod(Q_e, prod<mat>(E.P(), trans(Q_e)));
		}
		
		
		void RobotAbstract::computeStatePerturbation() {
			Q = jmath::ublasExtra::prod_JPJt(perturbation.P(), XNEW_pert);
//JFR_DEBUG("P " << perturbation.P());
//JFR_DEBUG("XNEW_pert " << XNEW_pert);
//JFR_DEBUG("Q " << Q);
		}


		void RobotAbstract::computeControls(double time1, double time2, jblas::mat & controls, bool release = true)
		{
			hardware::HardwareSensorProprioAbstract::VecIndT readings = hardwareEstimatorPtr->getRaws(time1, time2, release);
			unsigned data_vsize = readings(0).data.size();
			jblas::vec u(data_vsize-1), prev_u(data_vsize-1), next_u(data_vsize-1), control_tmp(data_vsize);
			controls.resize(readings.size()-1, data_vsize, false);

			jblas::ind_array instantArray = hardwareEstimatorPtr->instantValues()-1;
			jblas::ind_array incrementArray = hardwareEstimatorPtr->incrementValues()-1;
			jblas::vec_indirect u_instant(u, instantArray), prev_u_instant(prev_u, instantArray), next_u_instant(next_u, instantArray);
			jblas::vec_indirect u_increment(u, incrementArray), prev_u_increment(prev_u, incrementArray), next_u_increment(next_u, incrementArray);

			double a, cur_time = time1, after_time, prev_time = readings(0).data(0), next_time, average_time;
			prev_u = ublas::subrange(readings(0).data,1,data_vsize);

			size_t j = 0;
			for(size_t i = 0; i < readings.size(); i++)
			{
				next_time = after_time = readings(i).data(0);
				if (after_time > time2 || i == readings.size()-1) after_time = time2;
				if (after_time <= cur_time) continue;
				control_tmp(0) = after_time - cur_time;
				next_u = ublas::subrange(readings(i).data,1,data_vsize);

				average_time = (after_time+cur_time)/2; // middle of the integration interval
				if (next_time-prev_time < 1e-6) a = 0; else a = (average_time-prev_time)/(next_time-prev_time);
				u_instant = (1-a)*prev_u_instant + a*next_u_instant; // average command for the integration interval

				if (next_time-prev_time < 1e-6) a = 0; else a = (after_time-prev_time)/(next_time-prev_time);
				u_increment = a*next_u_increment;
				ublas::subrange(control_tmp, 1, control_tmp.size()) = u;
				if (j >= controls.size1()) controls.resize(j+1, controls.size2(), true); // should very rarely happen
				ublas::matrix_row<jblas::mat>(controls, j) = control_tmp;

				prev_time = cur_time = next_time;
				prev_u = next_u;
				++j;
			}
			if (j < controls.size1()) controls.resize(j, controls.size2(), true); // should very rarely happen
			dt_or_dx = time2 - time1;
		}


		void RobotAbstract::move(double time){
			bool firstmove = false;
			if (self_time < 0.) { firstmove = true; self_time = time; }
			if (hardwareEstimatorPtr)
			{
				if (firstmove) // compute average past control and allow the robot to init its state with it
				{
					hardware::HardwareSensorProprioAbstract::VecIndT readings = hardwareEstimatorPtr->getRaws(-1., time);
					self_time = 0.;
					dt_or_dx = 0.;
					unsigned nreadings = readings.size();
					if (readings(nreadings-1).data(0) >= time) nreadings--; // because it could be available offline but not online

					unsigned data_vsize = readings(0).data.size();
					jblas::vec avg_u(data_vsize-1); avg_u.clear();
					for(size_t i = 0; i < nreadings; i++)
						avg_u += ublas::subrange(readings(i).data,1,data_vsize);
					if (nreadings) avg_u /= nreadings;

					jblas::vec var_u(data_vsize-1); var_u.clear();
					jblas::vec diff_u(data_vsize-1);
					for(size_t i = 0; i < nreadings; i++) {
						diff_u = ublas::subrange(readings(i).data,1,data_vsize) - avg_u;
						var_u += ublas::element_prod(diff_u, diff_u);
					}
					if (nreadings) var_u /= nreadings;

					init(avg_u, var_u);
				}
				else // else just move with the available control
				{
					computeControls(self_time, time, controls, true);
					for(size_t i = 0; i < controls.size1(); ++i)
					{
						dt_or_dx = controls(i,0);
						perturbation.set_from_continuous(dt_or_dx);
						control = ublas::subrange(matrix_row<jblas::mat>(controls, i), 1, controls.size2());
						move();
					}
				}
			} else
			{
				dt_or_dx = time - self_time;
				perturbation.set_from_continuous(dt_or_dx);
				control.clear();
				move();
			}
			self_time = time;
		}


		void RobotAbstract::move_extrapolate() {
			vec xnew(state_extrapol_x.size());

			move_func(state_extrapol_x, control, perturbation.x(), dt_or_dx, xnew, XNEW_x, XNEW_pert);
			state_extrapol_x = xnew;

			if (!constantPerturbation) computeStatePerturbation();

			ind_array ia_inv = ia_set(0,0); // empty
			ind_array ia_state = ia_set(0, state.size());
			ixaxpy_prod(state_extrapol_P, ia_inv, XNEW_x, ia_state, ia_state, Q);
		}

		void RobotAbstract::move_extrapolate(double time)
		{
			boost::unique_lock<boost::mutex> l(mutex_extrapol);
			if (!extrapol_up_to_date)
			{
				self_time_extrapol = self_time_extrapol_init;
				state_extrapol_x = state_extrapol_x_init;
				state_extrapol_P = state_extrapol_P_init;
			}
			l.unlock();
			if (hardwareEstimatorPtr)
			{
				computeControls(self_time_extrapol, time, controls, false);
				for(size_t i = 0; i < controls.size1(); ++i)
				{
					dt_or_dx = controls(i,0);
					perturbation.set_from_continuous(dt_or_dx);
					control = ublas::subrange(matrix_row<jblas::mat>(controls, i), 1, control.size());
					move_extrapolate();
				}

			} else
			{
				dt_or_dx = time - self_time_extrapol;
				perturbation.set_from_continuous(dt_or_dx);
				control.clear();
				move_extrapolate();
			}
			self_time_extrapol = time;

		}

		void RobotAbstract::reinit_extrapolate()
		{
			boost::unique_lock<boost::mutex> l(mutex_extrapol);
			extrapol_up_to_date = false;
			self_time_extrapol_init = self_time;
			state_extrapol_x_init = state.x();
			state_extrapol_P_init = state.P();
		}


		void RobotAbstract::move_fake(double time){
			if (self_time < 0.) self_time = 0.;
			if (hardwareEstimatorPtr) hardwareEstimatorPtr->getRaws(self_time, time);
			self_time = time;
		}

		void RobotAbstract::getPos(double time, jblas::vec & x, jblas::vec & P)
		{
			move_extrapolate(time);

			jblas::vec euler_x(3);
			jblas::sym_mat euler_P(3,3);
			quaternion::q2e(ublas::subrange(state_extrapol_x, 3, 7), ublas::subrange(state_extrapol_P, 3,7, 3,7), euler_x, euler_P);
			ublas::subrange(x,0,3) = ublas::subrange(state_extrapol_x,0,3) + origin_sensors - origin_export;
			ublas::subrange(x,3,6) = euler_x;
			for(int i = 0; i < 3; ++i) P(i) = euler_P(i,i);
			for(int i = 0; i < 3; ++i) P(3+i) = euler_P(i,i);

		}


		void RobotAbstract::writeLogHeader(kernel::DataLogger& log) const
		{
			std::ostringstream oss; oss << "Robot " << id();
			log.writeComment(oss.str());
			log.writeLegendTokens("time");
			for(size_t i = 0; i < state.x().size(); ++i)
				{ oss.str(""); oss << "x" << i; log.writeLegend(oss.str()); }
			for(size_t i = 0; i < state.x().size(); ++i)
				{ oss.str(""); oss << "sig" << i; log.writeLegend(oss.str()); }
		}
		
		void RobotAbstract::writeLogData(kernel::DataLogger& log) const
		{
			log.writeData(self_time);
			for(size_t i = 0; i < state.x().size(); ++i)
				log.writeData(state.x()(i));
			for(size_t i = 0; i < state.x().size(); ++i)
				log.writeData(sqrt(state.P()(i,i)));
		}


	}
}
