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
			robot_pose(7), isOriginInit(false), origin(3),
			extrapol_up_to_date(false),
			control_extrapol(_size_control),
			perturbation_extrapol(_size_pert),
			XNEW_x_extrapol(_size_state, _size_state),
			XNEW_pert_extrapol(_size_state, _size_pert),
			Q_extrapol(_size_state, _size_state)
		{
			constantPerturbation = false;
			category = ROBOT;
			self_time = -1.;
			Q.clear();
			origin.clear();
			pose.x(quaternion::originFrame());
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
			robot_pose(7), isOriginInit(false), origin(3),
			extrapol_up_to_date(false),
			control_extrapol(_size_control),
			perturbation_extrapol(_size_pert),
			XNEW_x_extrapol(_size_state, _size_state),
			XNEW_pert_extrapol(_size_state, _size_pert),
			Q_extrapol(_size_state, _size_state)
		{
			constantPerturbation = true;
			category = ROBOT;
			self_time = -1.;
			origin.clear();
			pose.x(quaternion::originFrame());
			robot_pose.clear();
		}


		void RobotAbstract::setPoseStd(
			double x, double y, double z,
			double roll, double pitch, double yaw,
			double xStd, double yStd, double zStd,
			double rollStd, double pitchStd, double yawStd,
			bool degrees)
		{
			setPositionStd(x,y,z, xStd,yStd,zStd);
			setOrientationStd(roll,pitch,yaw, rollStd,pitchStd,yawStd, degrees);
		}

		void RobotAbstract::setPositionStd(
			double x, double y, double z,
			double xStd, double yStd, double zStd)
		{
			const double pos_[3] = { x, y, z };
			const double posStd_[3] = { xStd, yStd, zStd };

			ublas::subrange(pose.x(), 0, 3) = createVector<3> (pos_);
			subrange(pose.P(), 0,3, 0,3) = createSymMat<3>(posStd_);
		}

		void RobotAbstract::setOrientationStd(
			double roll, double pitch, double yaw,
			double rollStd, double pitchStd, double yawStd,
			bool degrees)
		{
			double euler_[3] = { roll, pitch, yaw };
			double eulerStd_[3] = { rollStd, pitchStd, yawStd };
			if (degrees) for(int i = 0; i < 3; ++i) { euler_[i] = jmath::degToRad(euler_[i]); eulerStd_[i] = jmath::degToRad(eulerStd_[i]); }

			// convert euler pose to quat pose
			vec3 euler = createVector<3> (euler_);
			ublas::subrange(pose.x(), 3, 7) = quaternion::e2q(euler);

			// convert euler uncertainty to quaternion uncertainty
			vec3 eulerStd = createVector<3> (eulerStd_);
			Gaussian E(3);	E.std(eulerStd);
			vec4 q;
			mat Q_e(4, 3);
			quaternion::e2q(euler, q, Q_e);
			subrange(pose.P(), 3,7, 3,7) = prod_JPJt(E.P(), Q_e); //prod(Q_e, prod<mat>(E.P(), trans(Q_e)));
		}

		void RobotAbstract::setRobotPose(jblas::vec6 const & pose_euler, bool degrees)
		{
			jblas::vec pose_euler_rad = pose_euler;
			if (degrees) for(int i = 3; i < 6; ++i) pose_euler_rad(i) = jmath::degToRad(pose_euler_rad(i));
			robot_pose = quaternion::e2q_frame(pose_euler_rad);
		}


		void RobotAbstract::setInitialPose(
			double x, double y, double z,
			double roll, double pitch, double yaw,
			double xStd, double yStd, double zStd,
			double rollStd, double pitchStd, double yawStd,
			bool degrees)
		{
			// mean
			double pose_euler_[6] = {x,y,z,roll,pitch,yaw};
			if (degrees) for(int i = 3; i < 6; ++i) pose_euler_[i] = jmath::degToRad(pose_euler_[i]);
			jblas::vec6 pose_euler = createVector<6>(pose_euler_);
			jblas::vec7 pose_quat = quaternion::e2q_frame(pose_euler);

			jblas::vec7 irobot_pose = quaternion::invertFrame(robot_pose);
			jblas::vec7 origin_pose = quaternion::composeFrames(pose_quat, irobot_pose);

			origin = subrange(origin_pose, 0,3);
			ublas::subrange(pose.x(), 3,7) = ublas::subrange(origin_pose, 3,7);
			isOriginInit = true;

			// cov
			double poseStd_euler_[6] = {xStd,yStd,zStd,rollStd,pitchStd,yawStd};
			if (degrees) for(int i = 3; i < 6; ++i) poseStd_euler_[i] = jmath::degToRad(poseStd_euler_[i]);
			jblas::vec6 poseStd_euler = createVector<6>(poseStd_euler_);
			Gaussian Pose_euler(6);	Pose_euler.std(poseStd_euler);

			mat Q_e(4, 3);
			quaternion::e2q_by_de(ublas::subrange(pose_euler, 3,6), Q_e);
			mat FQ_fe(7,6); FQ_fe.clear();
			ublas::subrange(FQ_fe,0,3,0,3) = identity_mat(3);
			ublas::subrange(FQ_fe, 3,7,3,6) = Q_e;

			jblas::mat O_p(7,7);
			quaternion::composeFrames_by_dglobal(pose_quat, irobot_pose, O_p);

			mat O_fe = ublas::prod(O_p, FQ_fe);
			subrange(pose.P(), 0,7, 0,7) = prod_JPJt(Pose_euler.P(), O_fe);

std::cout << "setInitialPose " << pose_euler << " std " << poseStd_euler << " : origin " << origin << " pose " << pose.x() << " cov " << pose.P() << std::endl;
		}


		void RobotAbstract::setInitialOrientation(
			double roll, double pitch, double yaw,
			double rollStd, double pitchStd, double yawStd,
			bool degrees)
		{
			// mean
			double ori_euler_[3] = {roll,pitch,yaw};
			if (degrees) for(int i = 0; i < 3; ++i) ori_euler_[i] = jmath::degToRad(ori_euler_[i]);
			jblas::vec3 ori_euler = createVector<3>(ori_euler_);
			jblas::vec4 ori_quat = quaternion::e2q(ori_euler);

			jblas::vec4 irobot_ori = quaternion::q2qc(ublas::subrange(robot_pose, 3, 7));
			ublas::subrange(pose.x(), 3,7) = quaternion::qProd(ori_quat, irobot_ori);

			// cov
			double oriStd_euler_[3] = {rollStd,pitchStd,yawStd};
			if (degrees) for(int i = 0; i < 3; ++i) oriStd_euler_[i] = jmath::degToRad(oriStd_euler_[i]);
			jblas::vec3 oriStd_euler = createVector<3>(oriStd_euler_);
			Gaussian Ori_euler(3);	Ori_euler.std(oriStd_euler);

			mat Q_e(4, 3);
			quaternion::e2q_by_de(ori_euler, Q_e);

			jblas::mat O_p(4,4);
			quaternion::qProd_by_dq1(ori_quat, O_p);

			mat O_e = ublas::prod(O_p, Q_e);
			subrange(pose.P(), 3,7, 3,7) = prod_JPJt(Ori_euler.P(), O_e);

std::cout << "setInitialOrientation " << ori_euler << " std " << oriStd_euler << " pose " << pose.x() << " cov " << pose.P() << std::endl;
		}


		void RobotAbstract::getCurrentPose(double time, jblas::vec & x, jblas::vec & P)
		{
			if (state_extrapol_x.size() == 0) { x = P = jblas::zero_vec(7); }
			move_extrapolate(time);
			slamPoseToRobotPose(ublas::subrange(state_extrapol_x,0,7), ublas::subrange(state_extrapol_P,0,7,0,7), x, P);
		}
		
		void RobotAbstract::computeStatePerturbation() {
			Q = jmath::ublasExtra::prod_JPJt(perturbation.P(), XNEW_pert);
//JFR_DEBUG("P " << perturbation.P());
//JFR_DEBUG("XNEW_pert " << XNEW_pert);
//JFR_DEBUG("Q " << Q);
		}

		void RobotAbstract::computeStatePerturbation_extrapol() {
			Q_extrapol = jmath::ublasExtra::prod_JPJt(perturbation_extrapol.P(), XNEW_pert_extrapol);
		}

		bool RobotAbstract::computeControls(double time1, double time2, jblas::mat & controls, bool release = true) const
		{
			hardware::HardwareSensorProprioAbstract::VecIndT readings = hardwareEstimatorPtr->getRaws(time1, time2, release);
			if (readings.size() < 2 || readings(readings.size()-1).data(0) < time2) return false;
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
			return true;
		}


		bool RobotAbstract::move(double time)
		{
			if (hardwareEstimatorPtr)
			{
				if (self_time < 0.) // first move, compute average past control and allow the robot to init its state with it
				{
					hardware::HardwareSensorProprioAbstract::VecIndT readings = hardwareEstimatorPtr->getRaws(-1., time);
					self_time = 0.;
					dt_or_dx = 0.;
					unsigned nreadings = readings.size();
					if (nreadings < 1 || readings(nreadings-1).data(0) < time) return false;

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
					if (!computeControls(self_time, time, controls, true)) return false;
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
				if (self_time < 0.) self_time = time;
				dt_or_dx = time - self_time;
				perturbation.set_from_continuous(dt_or_dx);
				control.clear();
				move();
			}
			self_time = time;
			return true;
		}


		void RobotAbstract::move_extrapolate() {
			vec xnew(state_extrapol_x.size());

			move_func(state_extrapol_x, control_extrapol, perturbation_extrapol.x(), dt_or_dx_extrapol, xnew, XNEW_x_extrapol, XNEW_pert_extrapol, 1);
			state_extrapol_x = xnew;

			if (!constantPerturbation) computeStatePerturbation_extrapol();

			ind_array ia_inv = ia_set(0,0); // empty
			ind_array ia_state = ia_set(0, state.size());
			ixaxpy_prod(state_extrapol_P, ia_inv, XNEW_x_extrapol, ia_state, ia_state, Q_extrapol);
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
				computeControls(self_time_extrapol, time, controls_extrapol, false);
				for(size_t i = 0; i < controls_extrapol.size1(); ++i)
				{
					dt_or_dx_extrapol = controls_extrapol(i,0);
					perturbation_extrapol.set_from_continuous(dt_or_dx_extrapol);
					control_extrapol = ublas::subrange(matrix_row<jblas::mat>(controls_extrapol, i), 1, control_extrapol.size()+1);
					move_extrapolate();
				}

			} else
			{
				dt_or_dx_extrapol = time - self_time_extrapol;
				perturbation_extrapol.set_from_continuous(dt_or_dx_extrapol);
				control_extrapol.clear();
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
