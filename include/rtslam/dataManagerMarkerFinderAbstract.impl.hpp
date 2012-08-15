/**
 * \file dataManagerMarkerFinderAbstract.impl.hpp
 *
 * \date 22/06/2011
 * \author Paul Molodowitch
 * \ingroup rtslam
 */

#include "rtslam/dataManagerMarkerFinderAbstract.hpp"

#include "jmath/jblas.hpp"
#include "rtslam/quatTools.hpp"

namespace jafar {
	namespace rtslam {
		using hardware::ReadingAbstract;

		DataManagerMarkerFinderAbstract::
		DataManagerMarkerFinderAbstract(float markerSize_)
			: markerSize(markerSize_)
		{}

		void DataManagerMarkerFinderAbstract::
		detectNew(raw_ptr_t data)
		{
			MarkerPtr markerP = detectMarker(data);
			if (markerP)
			{
				setSensorPoseFromMarker(markerP);
			}
		}

		void DataManagerMarkerFinderAbstract::
		setSensorPoseFromMarker(MarkerPtr newMarker)
		{
			// Marker gives marker pose in sensor space... however,
			// updatePoseFromAbsReading expects the sensor pose, in global space

			// In our case, the "desired" global space is the same as the marker
			// space - so we want the sensor pose in marker space... which is
			// just the inverse of marker pose in sensor space.
			jblas::vec7 tempPose = newMarker->pose();
			newMarker->pose() = quaternion::invertFrame(tempPose);
			updatePoseFromAbsReading(sensor(), *newMarker);
		}


//		/*
//		 * Helper func for updatePoseFromAbsReading
//		 */
//		template<class Vec, class Mat>
//		void setVarianceAsRatio(const Vec& values, double ratio, Mat& covariances)
//		{
//			size_t size = values.size();
//			if(size != covariances.size1())
//			{
//				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "setCovarianceAsRatio - size of vector did not match size of matrix");
//			}
//			if(size != covariances.size2())
//			{
//				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "setCovarianceAsRatio - matrix was not a square matrix");
//			}
//			covariances = jblas::zero_mat(size);
//
//			for(size_t i = 0; i < size; ++i)
//			{
//				covariances(i, i) = jmath::sqr(ratio * values[i]);
//			}
//		}

		/*
		 * Helper func for updatePoseFromAbsReading
		 */
		template<class Mat>
		void setVarianceAsConst(float value, Mat& covariances)
		{
			size_t size = covariances.size1();
			if(size != covariances.size2())
			{
				JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "setVarianceAsConst - matrix was not a square matrix");
			}
			covariances = jblas::zero_mat(size);

			for(size_t i = 0; i < size; ++i)
			{
				covariances(i, i) = value;
			}
		}

		// TODO: if want to merge with code in sensorAbsloc, have to add in
		// support for use_for_init and bundle observation
		void updatePoseFromAbsReading(SensorAbstract& sensor,
				const hardware::ReadingAbstract& reading)
		{
			//jblas::ind_array ia_global;
			// using sensor.ia_globalPose directly now...
			// ...but I think i can just use robot.pose.ia()
			// (it seems like sensor pose is generally treated like a constant)

			int innovationSize = reading.obsSize();
			int poseSize = sensor.ia_globalPose.size();

			Innovation innovation(innovationSize);
			Measurement measurement(innovationSize);
			Expectation expectation(innovationSize);

			jblas::mat expectationJacobian(innovationSize, poseSize);  // Jacobian for transform from prev state to expectation
			jblas::mat innovationJacobian(innovationSize, poseSize); // Jacobian for transform from prev state to innovation

			size_t indexE_OriEuler, indexE_Pos, indexE_OriQuat; /// indexes in expectation
			int indexD_Pos, indexD_OriEuler, indexD_OriQuat; /// indexes in reading data

			{
				// initialize type of data the hardware sensor is providing
				size_t indexE = 0; // current index when building expectation

				indexD_Pos = reading.getQuantity(ReadingAbstract::qPos);
				if (indexD_Pos >= 0) { indexE_Pos = indexE; indexE += reading.QuantityObsSizes[hardware::HardwareSensorProprioAbstract::qPos]; }

				indexD_OriQuat = reading.getQuantity(ReadingAbstract::qOriQuat);
				if (indexD_OriQuat >= 0) { indexE_OriQuat = indexE; indexE += reading.QuantityObsSizes[hardware::HardwareSensorProprioAbstract::qOriQuat]; }

				indexD_OriEuler = reading.getQuantity(ReadingAbstract::qOriEuler);
				if (indexD_OriEuler >= 0) { indexE_OriEuler = indexE; indexE += reading.QuantityObsSizes[hardware::HardwareSensorProprioAbstract::qOriEuler]; }
			}


			Gaussian& sensorPose = sensor.pose;
			RobotAbstract& robot = sensor.robot();
			Gaussian& robotPose = robot.pose;

			expectationJacobian.clear();
			expectation.P() = jblas::zero_mat(innovationSize);
			jblas::vec sensorOffsetRobot = ublas::subrange(sensorPose.x(), 0, 3);		// position of sensor, in robot-space
			jblas::vec sensorQuatRobot = ublas::subrange(sensorPose.x(), 3, 7);		// quat-rot of sensor, in robot-space
			jblas::vec robotPos = ublas::subrange(robotPose.x(), 0, 3);		// position of robot, in world-space
			jblas::vec robotQuat = ublas::subrange(robotPose.x(), 3, 7);		// quat-rot of robot, in world-space
			jblas::vec sensorOffsetWorld = quaternion::rotate(robotQuat, sensorOffsetRobot);			// offset vector from robot to sensor, in world-space

			// POSITION
			if (indexD_Pos >= 0)
			{
				jblas::mat sensorOffsetJacobian(3, 4);  // Jacobian for transform of sensor offset (from robot origin) from robot space to world space

				// compute expectation.x and expectationJacobian
				quaternion::rotate_by_dq(robotQuat, sensorOffsetRobot, sensorOffsetJacobian);  // Jacobian of transform from sensorOffsetRobot to sensorOffsetWorld
				ublas::subrange(expectationJacobian, indexE_Pos,indexE_Pos+3, 0,3) = jblas::identity_mat(3);
				ublas::subrange(expectationJacobian, indexE_Pos,indexE_Pos+3, 3,7) = sensorOffsetJacobian;
				ublas::subrange(expectation.x(), indexE_Pos,indexE_Pos+3) = robotPos + sensorOffsetWorld;

				// fill measurement
				ublas::vector_range<jblas::vec_indirect> measureX_pos = ublas::subrange(measurement.x(), indexE_Pos,indexE_Pos+3);
				ublas::matrix_range<jblas::sym_mat_indirect> measureP_pos = ublas::subrange(measurement.P(), indexE_Pos,indexE_Pos+3,indexE_Pos,indexE_Pos+3);
				measureX_pos = reading.getQuantityData(ReadingAbstract::qPos) - robot.origin;  // transfer position reading from raw data into measurement, correcting for robot origin
				switch (reading.covType()) {
					case hardware::HardwareSensorProprioAbstract::ctNone:
						//setVarianceAsRatio(measureX_pos, .05, measureP_pos);
						setVarianceAsConst(.0001, measureP_pos);
					case hardware::HardwareSensorProprioAbstract::ctVar :
						break;
					case hardware::HardwareSensorProprioAbstract::ctFull:
						break;
					default:
						break;
				}

				// TODO gating ?
			}

			// ORIENTATION QUAT
			if (indexD_OriQuat >= 0)
			{
				jblas::mat quatMultJacobian(4, 4);	// Jacobian for quat mult (robotQuat * sensorQuatRobot)

				// compute expectation.x and expectationJacobian
				jblas::vec4 sensorQuatWorld = quaternion::qProd(robotQuat, sensorQuatRobot);  // orientation of sensor, in world space
				quaternion::qProd_by_dq1(sensorQuatRobot, quatMultJacobian);	// Jacobian for transform from robotQuat to sensorQuatWorld
				ublas::subrange(expectation.x(), indexE_OriQuat,indexE_OriQuat+4) = sensorQuatWorld;
				ublas::subrange(expectationJacobian, indexE_OriQuat,indexE_OriQuat+4, 3,7) = quatMultJacobian;

				// fill measurement
				ublas::vector_range<jblas::vec_indirect> measureX_quat = ublas::subrange(measurement.x(), indexE_OriQuat,indexE_OriQuat+4);
				ublas::matrix_range<jblas::sym_mat_indirect> measureP_quat = ublas::subrange(measurement.P(), indexE_OriQuat,indexE_OriQuat+4,indexE_OriQuat,indexE_OriQuat+4);
				measureX_quat = reading.getQuantityData(ReadingAbstract::qOriQuat);

				switch (reading.covType()) {
					case hardware::HardwareSensorProprioAbstract::ctNone:
						//setVarianceAsRatio(measureX_quat, .05, measureP_quat);
						setVarianceAsConst(.0001, measureP_quat);
					case hardware::HardwareSensorProprioAbstract::ctVar :
						break;
					case hardware::HardwareSensorProprioAbstract::ctFull:
						break;
					default:
						break;
				}
			}
			// ORIENTATION EULER
			else if (indexD_OriEuler >= 0)
			{
				jblas::mat quatMultJacobian(4, 4);	// Jacobian for quat mult (robotQuat * sensorQuatRobot)
				jblas::mat eulerQuatJacobian(3, 4); // Jacobian for transform from quaternion to euler
				jblas::vec3 euler;

				// compute expectation.x and expectationJacobian
				jblas::vec4 sensorQuatWorld = quaternion::qProd(robotQuat, sensorQuatRobot);  // orientation of sensor, in world space
				quaternion::qProd_by_dq1(sensorQuatRobot, quatMultJacobian);	// Jacobian for transform from robotQuat to sensorQuatWorld
				quaternion::q2e(sensorQuatWorld, euler, eulerQuatJacobian);
				ublas::subrange(expectation.x(), indexE_OriEuler,indexE_OriEuler+3) = euler;
				ublas::subrange(expectationJacobian, indexE_OriEuler,indexE_OriEuler+3, 3,7) = ublas::prod(eulerQuatJacobian, quatMultJacobian);

				// fill measurement
				ublas::vector_range<jblas::vec_indirect> measureX_euler = ublas::subrange(measurement.x(), indexE_OriEuler,indexE_OriEuler+3);
				ublas::matrix_range<jblas::sym_mat_indirect> measureP_euler = ublas::subrange(measurement.P(), indexE_OriEuler,indexE_OriEuler+3,indexE_OriEuler,indexE_OriEuler+3);
				measureX_euler = reading.getQuantityData(ReadingAbstract::qOriEuler);

				switch (reading.covType()) {
					case hardware::HardwareSensorProprioAbstract::ctNone:
						//setVarianceAsRatio(measureX_euler, .05, measureP_euler);
						setVarianceAsConst(.01, measureP_euler);
					case hardware::HardwareSensorProprioAbstract::ctVar :
						break;
					case hardware::HardwareSensorProprioAbstract::ctFull:
						break;
					default:
						break;
				}
			}

			// compute expectation.P and innovation
			expectation.P() += ublasExtra::prod_JPJt(ublas::project(robot.mapPtr()->filterPtr->P(), sensor.ia_globalPose, sensor.ia_globalPose), expectationJacobian);
			innovation.x() = measurement.x() - expectation.x();
			innovation.P() = measurement.P() + expectation.P();
			innovationJacobian = -expectationJacobian;

			map_ptr_t mapPtr = robot.mapPtr();
			ind_array ia_x = mapPtr->ia_used_states();
			mapPtr->filterPtr->correct(ia_x,innovation,innovationJacobian,sensor.ia_globalPose);
		}

	} // namespace ::rtslam
} // namespace jafar::

