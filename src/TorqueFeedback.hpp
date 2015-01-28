/** TorqueFeedback.hpp
*
* @class TorqueFeedback
*
* \author Max Baeten
* \date January, 2014
* \version 1.0
*
*/
#ifndef TORQUEFEEDBACK_HPP
#define TORQUEFEEDBACK_HPP

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

namespace FORCECONTROL
{
	typedef std::vector<double> doubles;

	class TorqueFeedback
	: public RTT::TaskContext
	{
		private:
			uint N;

			doubles input_u;
			doubles input_tau;
			doubles input_taudot;
			doubles output;
			int cntr;
			bool safe;

			doubles MotorInertiaSuperDiagonal, MotorInertiaDiagonal, MotorInertiaSubDiagonal;
			doubles ScaledMotorInertiaSuperDiagonal, ScaledMotorInertiaDiagonal, ScaledMotorInertiaSubDiagonal;
			doubles DampingSuperDiagonal, DampingDiagonal, DampingSubDiagonal;
			doubles StiffnessSuperDiagonal, StiffnessDiagonal, StiffnessSubDiagonal;
			doubles OUTU, OUTTAU, OUTTAUDOT;

			// vectors
			Eigen::VectorXd u, tau, taudot;				// input vectors
			Eigen::VectorXd output_u, output_tau, output_taudot; 						// output vectors

			// matrices
			Eigen::MatrixXd	DKinv, BBtinv, IMinBBtinv; 	// intermediate matrices

		public:
			InputPort<doubles> inport_u;
			InputPort<doubles> inport_tau;
			InputPort<doubles> inport_taudot;
			InputPort<bool> enable_inport;
			OutputPort<doubles> outport_u;
			OutputPort<doubles> outport_tau;
			OutputPort<doubles> outport_taudot;

			TorqueFeedback(const string& name);
			~TorqueFeedback();

			bool configureHook();
			bool startHook();
			void updateHook();
	};
}
#endif

