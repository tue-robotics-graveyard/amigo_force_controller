/** UPF_TorqueFeedbackController.hpp
*
* @class TorqueFeedbackController
*
* \author Max Baeten
* \date January, 2014
* \version 1.0
*
*/
#ifndef UPF_TORQUEFEEDBACKCONTROLLER_HPP
#define UPF_TORQUEFEEDBACKCONTROLLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>
#include <math.h>

#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>

using namespace Eigen;
using namespace std;
using namespace RTT;

namespace UPF
{
	typedef std::vector<double> doubles;

	class TorqueFeedbackController
	: public RTT::TaskContext
	{
		private:
			uint N;

			doubles input_u;
			doubles input_tau;
			doubles input_taudot;
			doubles output;

			doubles MotorInertiaSuperDiagonal, MotorInertiaDiagonal, MotorInertiaSubDiagonal;
			doubles ScaledMotorInertiaSuperDiagonal, ScaledMotorInertiaDiagonal, ScaledMotorInertiaSubDiagonal;
			doubles DampingSuperDiagonal, DampingDiagonal, DampingSubDiagonal;
			doubles StiffnessSuperDiagonal, StiffnessDiagonal, StiffnessSubDiagonal;

			// vectors
			Eigen::VectorXd u, tau, taudot;				// input vectors
			Eigen::VectorXd Tau_m; 						// output vector

			// matrices
			Eigen::MatrixXd	DKinv, BBtinv, IMinBBtinv; 	// intermediate matrices

		public:
			InputPort<doubles> inport_u;
			InputPort<doubles> inport_tau;
			InputPort<doubles> inport_taudot;
			OutputPort<doubles> outport;

			TorqueFeedbackController(const string& name);
			~TorqueFeedbackController();

			bool configureHook();
			bool startHook();
			void updateHook();
	};
}
#endif

