/** MotorCharacteristics.hpp
 *
 * @class MotorCharacteristics
 *
 * \author Max Baeten
 * \date September, 2014
 * \version 1.0
 *
 */

#ifndef MOTORCHARACTERISTICS_HPP
#define MOTORCHARACTERISTICS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace std;
using namespace RTT;

namespace FORCECONTROL
{
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that calculates a voltage from a desired current
   * Use this component for hardware that is voltage controlled where 
   * current control is desired.
   *
   * The component has one input port that should receive two vectors of
   * doubles. One containing the rotational speed of the motor and one 
   * containing the desired current.
   * 
   * The component first calculates the derivative of the input and than
   * calculates via Va = Ke*Wm + Ra*Ia + La*dIa/dt
   */
   
  class MotorCharacteristics
  : public RTT::TaskContext
    {
    private:

		// Input and output ports
		InputPort<doubles> inport_current;
		InputPort<doubles> inport_position;
		OutputPort<doubles> outport;

		// global
        doubles previous_output_current;
        doubles previous_input_current;
        doubles previous_output_position;
        doubles previous_input_position;
        long double dt;
        long double old_time;
        std::vector<double> a;
        std::vector<double> b;

		// Properties
		uint N;
		double Ts;
		doubles Ke;
		doubles gearratio;
		doubles Ra;
		doubles La;
        doubles voltage_gains;
		
		// Declaring private functions
		void determineDt();
        doubles calculatederivative_current(doubles diff_in);
        doubles calculatederivative_position(doubles diff_in);
		
    public:

		MotorCharacteristics(const string& name);
		~MotorCharacteristics();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
