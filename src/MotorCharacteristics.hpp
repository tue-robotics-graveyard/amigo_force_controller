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

template <class T>
inline string to_string (const T& t){
    stringstream ss;
    ss << t;
    return ss.str();
};

namespace FORCECONTROL
{
    typedef vector<double> doubles;

    /** @brief A Component that calculates pwm values to realise a desired 
    torque. 

    1) The torque inputs of all inports[i] are added
    2) Torques converted into a current
    3) Derivatives of the current and position are then determined 
    4) Desired voltage is calculated via Va = Ke*Wm + Ra*Ia + La*dIa/dt
    5) Voltage is converted into a PWM value
    */

    class MotorCharacteristics
    : public RTT::TaskContext
    {
        private:
            // Input and output ports
            InputPort<doubles> inports[3];
            InputPort<doubles> inport_position;
            OutputPort<doubles> outport1;
            OutputPort<doubles> outport2;
            OutputPort<doubles> outport3;
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
            uint Nin;
            double Ts;
            doubles Ke;
            doubles gearratio;
            doubles Ra;
            doubles La;
            doubles Volt2PWM;

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
