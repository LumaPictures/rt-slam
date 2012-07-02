/**
 * \file robotInertial.cpp
 * \date 26/03/2010
 * \author jsola
 * \ingroup rtslam
 */

#include "jmath/jblas.hpp"

//#include "rtslam/robotAbstract.hpp"
#include "rtslam/robotInertial.hpp"

#include "rtslam/quatTools.hpp"

namespace jafar {
	namespace rtslam {
		using namespace std;
		using namespace jblas;
		using namespace ublas;
		using namespace quaternion;


#if ESTIMATE_G_VEC
			int RobotInertial::g_size = 3;
#else
			int RobotInertial::g_size = 1;
#endif


		/**
		 * Remote constructor from remote map.
		 * \param _map the remote map
		 */
		RobotInertial::RobotInertial(const map_ptr_t & _mapPtr) :
			RobotAbstract(_mapPtr, RobotInertial::size(), RobotInertial::size_control(), RobotInertial::size_perturbation())
		{
			constantPerturbation = false;
			type = INERTIAL;
			z_axis.clear(); z_axis(2) = 1;
		}
		RobotInertial::RobotInertial(const simulation_t dummy, const map_ptr_t & _mapPtr) :
			RobotAbstract(FOR_SIMULATION, _mapPtr, RobotInertial::size(), RobotInertial::size_control(), RobotInertial::size_perturbation()) {
			constantPerturbation = true;
			type = INERTIAL;
		}


		/*
		 * This motion model is driven by IMU measurements and random perturbations, and defined by:
		 * The state vector, x = [p q v ab wb g] , of size 19.
		 *
		 * The transition equation is
		 * - x+ = move_func(x,u,n),
		 *
		 * with u = [am, wm] the IMU measurements (the control input)
		 *  and n = [vi, ti, abi, wbi] the perturbation impulse.
		 *
		 * the transition equation f() is decomposed as:
		 * #if AVGSPEED
		 * - p+  = p + (v + v+)/2*dt
		 * #else
		 * - p+  = p + v*dt
		 * #endif
		 * - q+  = q**((wm - wb)*dt + ti)           <-- ** : quaternion product ; ti : theta impulse
		 * - v+  = v + (R(q)*(am - ab) + g)*dt + vi <-- am and wm: IMU measurements ; vi : v impulse
		 * - ab+ = ab + abi                         <-- abi : random walk in acc bias with abi impulse perturbation
		 * - wb+ = wb + wbi                         <-- wbi : random walk of gyro bias with wbi impulse perturbation
		 * - g+  = g                                <-- g  : gravity vector, constant but unknown
		 * -----------------------------------------------------------------------------
		 *
		 * The Jacobian XNEW_x is built with
		 *   var    |  p       q        v        ab       wb       g
		 *      pos |  0       3        7        10       13       16
		 *   -------+----------------------------------------------------
		 * #if AVGSPEED
		 *   p   0  |  I  VNEW_q*dt/2  I*dt -R*dt*dt/2    0     I*dt*dt/2
		 * #else
		 *   p   0  |  I       0       I*dt      0        0        0
		 * #endif
		 *   q   3  |  0     QNEW_q     0        0      QNEW_wb    0
		 *   v   7  |  0     VNEW_q     I      -R*dt      0       I*dt
		 *   ab  10 |  0       0        0        I        0        0
		 *   wb  13 |  0       0        0        0        I        0
		 *   g   16 |  0       0        0        0        0        I
		 * -----------------------------------------------------------------------------
		 *
		 * The Jacobian XNEW_pert is built with
		 *   var    |  vi    ti   abi  wbi
		 *      pos |  0     3     6     9
		 *   -------+----------------------
		 * #if AVGSPEED
		 *   p   0  |I.dt/2  0     0     0
		 * #else
		 *   p   0  |  0     0     0     0
		 * #endif
		 *   q   3  |  0  QNEW_ti  0     0
		 *   v   7  |  I     0     0     0
		 *   ab  10 |  0     0     I     0
		 *   wb  13 |  0     0     0     I
		 *   g   16 |  0     0     0     0
		 * -----------------------------------------------------------------------------
		 */
		void RobotInertial::move_func(const vec & _x, const vec & _u, const vec & _n, double _dt, vec & _xnew, mat & _XNEW_x,
		    mat & _XNEW_pert) {

			// Separate things out to make it clearer
			vec3 p, v, ab, wb, gv;
			vec g;
			vec4 q;
			splitState(_x, p, q, v, ab, wb, g); // split state vector
			if (g_size == 1) { gv = z_axis * g(0); } else { gv = g; }

			// Split control and perturbation vectors into
			// sensed acceleration and sensed angular rate
			// and noises
			vec3 am, wm, vi, ti, abi, wbi; // measurements and random walks
			splitControl(_u, am, wm);
			splitPert(_n, vi, ti, abi, wbi);

			// Get new state vector
			vec3 pnew, vnew, abnew, wbnew;
			vec gnew;
			vec4 qnew;

			// It is useful to start obtaining a nice rotation matrix and the product R*dt
			Rdt = q2R(q) * _dt;

			// Invert sensor functions. Get true angular rates:
			// w = wsens - wb
			vec3 wtrue = wm - wb;

			// qnew = q x q(w * dt) (keep qwdt for later use)
			vec4 qwdt = v2q(wtrue * _dt + ti);
			qnew = qProd(q, qwdt); //    orientation

			// Invert sensor functions. Get true acceleration:
			// a = R(q)(asens - ab) + g
			vec3 atrue = rotate(qnew, (am - ab)) + gv;

			vnew = v + atrue * _dt + vi; //    velocity
			#if AVGSPEED
			pnew = p + (v+vnew)/2 * _dt; //     position
			#else
			pnew = p + v * _dt; //     position
			#endif
			abnew = ab + abi; //          acc bias
			wbnew = wb + wbi; //          gyro bias
			gnew = g; //                 gravity does not change
			
			// normalize quaternion
			ublasExtra::normalizeJac(qnew, QNORM_qnew);
			ublasExtra::normalize(qnew);

			// Put it all together - this is the output state
			unsplitState(pnew, qnew, vnew, abnew, wbnew, gnew, _xnew);


			// Now on to the Jacobian...
			// Identity is a good place to start since overall structure is like this
			// var    |  p       q        v        ab       wb       g
			//    pos |  0       3        7        10       13       16
			// -------+----------------------------------------------------
			//#if AVGSPEED
			// p   0  |  I  VNEW_q*dt/2  I*dt -R*dt*dt/2    0     I*dt*dt/2
			//#else
			// p   0  |  I       0       I*dt      0        0        0
			//#endif
			// q   3  |  0     QNEW_q     0        0      QNEW_wb    0
			// v   7  |  0     VNEW_q     I      -R*dt      0       I*dt
			// ab  10 |  0       0        0        I        0        0
			// wb  13 |  0       0        0        0        I        0
			// g   16 |  0       0        0        0        0        I

			_XNEW_x.assign(identity_mat(state.size()));

			// Fill in XNEW_v: VNEW_g and PNEW_v = I * dt
			identity_mat I(3);
			Idt = I * _dt;
			mat Iz; if (g_size == 1) { Iz.resize(3,1); Iz.clear(); Iz(2,0)=1; } else { Iz = I; }
			mat Izdt = Iz * _dt;
			subrange(_XNEW_x, 0, 3, 7, 10) = Idt;
			#if AVGSPEED
			subrange(_XNEW_x, 0, 3, 16, 16+g_size) = Izdt*_dt/2;
			#endif
			subrange(_XNEW_x, 7, 10, 16, 16+g_size) = Izdt;

			// Fill in QNEW_q
			// qnew = qold ** qwdt  ( qnew = q1 ** q2 = qProd(q1, q2) in rtslam/quatTools.hpp )
			qProd_by_dq1(qwdt, QNEW_q);
			subrange(_XNEW_x, 3, 7, 3, 7) = prod(QNORM_qnew, QNEW_q);

			// Fill in QNEW_wb
			// QNEW_wb = QNEW_qwdt * QWDT_wdt * WDT_w * W_wb
			//         = QNEW_qwdt * QWDT_w * W_wb
			//         = QNEW_qwdt * QWDT_w * (-1)
			qProd_by_dq2(q, QNEW_qwdt);
			// Here we get the derivative of qwdt wrt wtrue, so we consider dt = 1 and call for the derivative of v2q() with v = w*dt
//			v2q_by_dv(wtrue, QWDT_w);
			v2q_by_dv(wtrue*_dt, QWDT_w); QWDT_w *= _dt;
			QNEW_w = prod ( QNEW_qwdt, QWDT_w);
			subrange(_XNEW_x, 3, 7, 13, 16) = -prod(QNORM_qnew,QNEW_w);

			// Fill VNEW_q
			// VNEW_q = d(R(q)*v) / dq
			rotate_by_dq(q, am-ab, VNEW_q); VNEW_q *= _dt;
			subrange(_XNEW_x, 7, 10, 3, 7) = VNEW_q;
			#if AVGSPEED
			subrange(_XNEW_x, 0, 3, 3, 7) = VNEW_q*_dt/2;
			#endif

			// Fill in VNEW_ab
			subrange(_XNEW_x, 7, 10, 10, 13) = -Rdt;
			#if AVGSPEED
			subrange(_XNEW_x, 0, 3, 10, 13) = -Rdt*_dt/2;
			#endif


			// Now on to the perturbation Jacobian XNEW_pert

			// Form of Jacobian XNEW_pert
			// It is like this:
			// var    |  vi    ti    abi    wbi
			//    pos |  0     3     6     9
			// -------+----------------------
			//#if AVGSPEED
			// p   0  |I.dt/2  0     0     0
			//#else
			// p   0  |  0     0     0     0
			//#endif
			// q   3  |  0   QNEW_ti 0     0
			// v   7  |  I     0     0     0
			// ab  10 |  0     0     I     0
			// wb  13 |  0     0     0     I
			// g   16 |  0     0     0     0

			// Fill in the easy bits first
			_XNEW_pert.clear();
			#if AVGSPEED
			ublas::subrange(_XNEW_pert, 0, 3, 0, 3) = Idt/2;
			#endif
			ublas::subrange(_XNEW_pert, 7, 10, 0, 3) = I;
			ublas::subrange(_XNEW_pert, 10, 13, 6, 9) = I;
			ublas::subrange(_XNEW_pert, 13, 16, 9, 12) = I;

			// Tricky bit is QNEW_ti = d(qnew)/d(ti)
			// Here, ti is the integral of the perturbation, ti = integral_{tau=0}^dt (wn(t) * dtau),
			// with: wn: the angular rate measurement noise
			//       dt: the integration period
			//       ti: the resulting angular impulse
			// The integral of the dynamic equation is:
			// q+ = q ** v2q((wm - wb)*dt + ti)
			// We have: QNEW_ti = QNEW_w / dt
			//    with: QNEW_w computed before.
			// The time dependence needs to be included in perturbation.P(), proportional to perturbation.dt:
			//   U = perturbation.P() = U_continuous_time * dt
			//	with: U_continuous_time expressed in ( rad / s / sqrt(s) )^2 = rad^2 / s^3 <-- yeah, it is confusing, but true.
			//   (Use perturbation.set_P_from_continuous() helper if necessary.)
			//
			subrange(_XNEW_pert, 3, 7, 3, 6) = prod (QNORM_qnew, QNEW_w) * (1 / _dt);
		}

#if 1
		vec RobotInertial::e_from_g(const vec3 & _g)
		{
			double norm_g = ublas::norm_2(_g);
			vec3 gf; gf.clear(); gf(2) = -norm_g;
			vec3 rotv = ublasExtra::crossProd(_g,gf);
			double norm_rotv = ublas::norm_2(rotv);
			double sin_a = norm_rotv / (norm_g*norm_g);
			double cos_a = ublas::inner_prod(gf,_g) / (norm_g*norm_g);
			double a = atan2(sin_a, cos_a);
			rotv = a/norm_rotv * rotv;
			return quaternion::q2e(quaternion::v2q(rotv));
		}
#else
		vec RobotInertial::e_from_g(const vec3 & _g)
		{
			vec3 xr, yr, zr, xw, yw, zw; // robot and world frame axis
			xw.clear(); xw(0)=1.; yw.clear(); yw(1)=1.; zw.clear(); zw(2)=1.;
			zr = -_g/ublas::norm_2(_g);
			yr = ublasExtra::crossProd(zr,xw); if (yr(0) < 0.0) yr = -yr;
			xr = -ublasExtra::crossProd(yw,zr); if (xr(0) < 0.0) xr = -xr;
			if (ublas::norm_2(xr) > ublas::norm_2(yr)) // just in case one of them is too close to 0
			{
				xr = xr / ublas::norm_2(xr);
				yr = ublasExtra::crossProd(zr,xr);
			} else
			{
				yr = yr / ublas::norm_2(yr);
				xr = ublasExtra::crossProd(yr,zr);
			}
			mat33 rot;
			rot(0,0)=xr(0); rot(1,0)=xr(1), rot(2,0)=xr(2);
			rot(0,1)=yr(0); rot(1,1)=yr(1), rot(2,1)=yr(2);
			rot(0,2)=zr(0); rot(1,2)=zr(1), rot(2,2)=zr(2);
			vec4 q = q2qc(R2q(rot));
			return q2e(q);
		}
#endif

		void RobotInertial::init_func(const vec & _x, const vec & _u, const vec & _U, vec & _xnew) {
			
			// Separate things out to make it clearer
			vec3 p, v, ab, wb, gv;
			vec g;
			vec4 q;
			splitState(_x, p, q, v, ab, wb, g); // split state vector

			// Split control vector into
			// sensed acceleration and sensed angular rate
			vec3 am, wm;
			splitControl(_u, am, wm);
			
			// init direction and uncertainty of g from acceleration
			const double th_g = 9.81;
			gv = th_g*(-am+ab)/ublas::norm_2(am-ab);
//			double G = (ublas::norm_2(am) - th_g); G = 2*G*G;
//			for (size_t i = pose.size() + 9; i < pose.size() + 9 + g_size; i++){
//				state.P(i,i) = std::max(state.P(i,i), G);
//			}
			if (g_size == 1) { g(0) = -th_g; } else { g = gv; }

			// init orientation from g
			#if INIT_Q_FROM_G
			vec3 e = e_from_g(gv);
			e(2) = q2e(q)(2); // restore initial yaw
			q = e2q(e);
			
			if (g_size == 1) { g(0) = -th_g; } else { g = - z_axis * th_g; } // update g

			vec3 estd; estd.clear();
			if (g_size == 1) // compute attitude initial uncertainty
			{
				vec3 av, wv;
				splitControl(_U, av, wv);

#if 1
				// roughly approximate the euler uncertainty because the exact jacobian is too complicated
				double uncert = 0.;
				for(int i = 0; i < 3; ++i) uncert += av(i)+state.P(10+i,10+i);
				uncert = sqrt(uncert/3);
				estd(0) = estd(1) = asin(uncert / 9.81);
#else
				// not working correctly, messes up with yaw and dometimes results in uncertainties too large
				// numerically approximate the euler uncertainty because the exact jacobian is too complicated
				std::cout << "gv " << gv << std::endl;
				vec3 gext;
				for(int i = 0; i < 3; ++i)
				{
					gext = gv;
					std::cout << "uncert " << i << " noise " << sqrt(av(i)) << " bias " << sqrt(state.P(10+i,10+i)) << std::endl;
					gext(i) += sqrt(av(i) + state.P(10+i,10+i)); // uncertainty of measure + bias
					std::cout << "gext " << gext << std::endl;
					vec3 eext = e_from_g(gext);
					std::cout << "eext " << eext << std::endl;
					for(int j = 0; j < 2; ++j) estd(j) = std::max(estd(j), std::abs(eext(j)-e(j)));
				}
#endif
				// now convert to quaternion uncertainty
				mat33 E; E.clear(); for(int j = 0; j < 2; ++j) E(j,j) = estd(j)*estd(j);
				mat Q_e(4,3);
				quaternion::e2q_by_de(e, Q_e);
				ublas::subrange(state.P(), 3,7, 3,7) = jmath::ublasExtra::prod_JPJt(E, Q_e);
			}
			#endif
			
			std::cout << "IMU initializes robot state with g = " << g << " (std " << sqrt(state.P(pose.size()+9,pose.size()+9)) <<
				") and orientation q = " << q << " e = " << quaternion::q2e(q) << std::endl;
#if INIT_Q_FROM_G
			if (g_size == 1) std::cout << "Euler attitude is " << e << " (std " << estd << ")" << std::endl;
#endif
			unsplitState(p, q, v, ab, wb, g, _xnew);
		}
		
		
		void RobotInertial::writeLogHeader(kernel::DataLogger& log) const
		{
			std::ostringstream oss; oss << "Robot " << id();
			log.writeComment(oss.str());

			log.writeLegendTokens("time");
			
			log.writeLegendTokens("absx absy absz");
			log.writeLegendTokens("absyaw abspitch absroll");
			log.writeLegendTokens("x y z");
			log.writeLegendTokens("qw qx qy qz");
			log.writeLegendTokens("vx vy vz");
			log.writeLegendTokens("axb ayb azb");
			log.writeLegendTokens("vyawb vpitchb vrollb");
			log.writeLegendTokens("gx gy gz");
			
			log.writeLegendTokens("sig_absx sig_absy sig_absz");
			log.writeLegendTokens("sig_absyaw sig_abspitch sig_absroll");
			log.writeLegendTokens("sig_x sig_y sig_z");
			log.writeLegendTokens("sig_qw sig_qx sig_qy sig_qz");
			log.writeLegendTokens("sig_vx sig_vy sig_vz");
			log.writeLegendTokens("sig_axb sig_ayb sig_azb");
			log.writeLegendTokens("sig_vyawb sig_vpitchb sig_vrollb");
			log.writeLegendTokens("sig_gx sig_gy sig_gz");
		}
		
		void RobotInertial::writeLogData(kernel::DataLogger& log) const
		{
			jblas::vec state_x(6), state_P(6);
			slamPoseToRobotPose(ublas::subrange(state.x(),0,7), ublas::subrange(state.P(),0,7,0,7), state_x, state_P);

			log.writeData(self_time);
			for(int i = 0 ; i < 3 ; ++i) log.writeData(state_x(i));
			for(int i = 0 ; i < 3 ; ++i) log.writeData(state_x(3+2-i));
			for(int i = 0 ; i < 7 ; ++i) log.writeData(state.x()(i));
			for(int i = 7 ; i < 10; ++i) log.writeData(state.x()(i));
			for(int i = 10; i < 13; ++i) log.writeData(state.x()(i));
			for(int i = 13; i < 16; ++i) log.writeData(state.x()(2-(i-13)+13));
			if (g_size == 1) { for(int i = 16; i < 16+2; ++i) log.writeData(0.); log.writeData(state.x()(16)); }
									else { for(int i = 16; i < 16+g_size; ++i) log.writeData(state.x()(i)); }

			
			for(int i = 0 ; i < 3 ; ++i) log.writeData(state_P(i));
			for(int i = 0 ; i < 3 ; ++i) log.writeData(state_P(3+2-i));
			for(int i = 0 ; i < 7 ; ++i) log.writeData(sqrt(state.P()(i,i)));
			for(int i = 7 ; i < 10; ++i) log.writeData(sqrt(state.P()(i,i)));
			for(int i = 10; i < 13; ++i) log.writeData(sqrt(state.P()(i,i)));
			for(int i = 13; i < 16; ++i) log.writeData(sqrt(state.P()(2-(i-13)+13,2-(i-13)+13)));
			if (g_size == 1) { for(int i = 16; i < 16+2; ++i) log.writeData(0.); log.writeData(sqrt(state.P()(16,16))); }
									else { for(int i = 16; i < 16+g_size; ++i) log.writeData(sqrt(state.P()(i,i))); }
		}

	}
}
