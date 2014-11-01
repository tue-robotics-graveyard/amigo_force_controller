/** MotorCharacteristics.cpp
 *
 * @class MotorCharacteristics
 *
 * \author Max Baeten
 * \date September, 2014
 * \version 1.0
 * To Do: Fix output of derivative, make derivative such that it can be called twice
 * or make derivative as port
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "MotorCharacteristics.hpp"

using namespace std;
using namespace RTT;
using namespace FORCECONTROL;

MotorCharacteristics::MotorCharacteristics(const string& name) : 
	    TaskContext(name, PreOperational),
	    N(0), Ts(0.0)
{
    addProperty( "TS", Ts ).doc("A double value that specifies the sampling time. Should match the sampling time of the component that triggers it (EventPort)");
    addProperty( "vector_size", N ).doc("An unsigned integer that specifies the vector size of the in and outputs.");
    addProperty( "NumberOfInports", Nin ).doc("An unsigned integer that specifies the number of the in.");
    addProperty( "MotorVoltageConstant", Ke ).doc("A vector containing motor voltage constants");
    addProperty( "GearRatio", gearratio ).doc("A vector containing gear ratios");
    addProperty( "TerminalResistance", Ra ).doc("A vector containing terminal resistances");
    addProperty( "ArmatureWindingInductance", La ).doc("A vector containing armature winding inductances");
    addProperty( "Volt2PWM", Volt2PWM ).doc("A vector containing gains to multiply voltage with for example for PWM controlled motors");
}

MotorCharacteristics::~MotorCharacteristics(){}

bool MotorCharacteristics::configureHook()
{
    if ( Nin > 3 ) {
        log(Error)<<"MotorCharacteristics: Nin larger than 3 is currently not supported. If you need more, increase the number of ports in the MotorCharacteristics.hpp!"<<endlog();
        return false;
    }

    // Adding inports
    addEventPort( "in1_ev", inports[0] );
    for (uint j = 1; j < Nin; j++) {
        addPort( "in"+to_string(j+1), inports[j] );
    }

    addEventPort( "in_position", inport_position );
    addPort( "out", outport );

    previous_input_current.resize(N);
    previous_output_current.resize(N);
    previous_input_position.resize(N);
    previous_output_position.resize(N);
    a.resize(2);
    b.resize(2);

    // Filter coefficient N
    double N = 100.0;
    double x = N*Ts+1;

    a[0] =   1;
    a[1] =  -1 / x;
    b[0] =  N / x;
    b[1] = -N / x;

    return true;
}

bool MotorCharacteristics::startHook()
{
    // Check Ports:
    for (uint j = 1; j < Nin; j++) {
        if ( !inports[j].connected() ) {
            log(Error)<<"MotorCharacteristics: One of the input ports not connected!"<<endlog();
            return false;
        }
    }
    if ( !outport.connected() || !inport_position.connected()) {
        log(Warning)<<"MotorCharacteristics: Output port or inport_position not connected!"<<endlog();
    }

    // Check Properties
    if (N < 1 || Ts <= 0) {
        log(Error)<<"MotorCharacteristics: N,Ts parameters not valid!"<<endlog();
        return false;
    }

    if (Ke.size() != N || gearratio.size() != N || Ra.size() != N || La.size() != N ) {
        log(Error)<<"MotorCharacteristics: MotorVoltageConstant, GearRatio, TerminalResistance, ArmatureWindingInductance parameters wrongly sized!"<<endlog();
        return false;
    }
    for (uint i = 0; i < N; i++) {
        if (Ke[i] < 0.0 || gearratio[i] < 0.0 || Ra[i] < 0.0 || La[i] < 0.0 ) {
            log(Error)<<"MotorCharacteristics: MotorVoltageConstant, GearRatio, TerminalResistance, ArmatureWindingInductance parameters erroneus parameters!"<<endlog();
            return false;
        }
    }

    for (uint i = 0; i < N; i++) {
        previous_input_current[i] = 0.0;
        previous_output_current[i] = 0.0;
        previous_input_position[i] = 0.0;
        previous_output_position[i] = 0.0;
    }

    return true;
}

void MotorCharacteristics::updateHook()
{
    // Read position input
    doubles position(N,0.0);
    inport_position.read(position);

    // Read and add all input torques
    doubles torque(N,0.0);
    for (uint j = 0; j < Nin; j++) {
        doubles input_torque(N,0.0);
        inports[j].read(input_torque);
        for (uint i = 0; i < N; i++) {
            torque[i]+= input_torque[i];
        }
    }

    // Calculate desired Current
    doubles current(N,0.0);
    for (uint i = 0; i < N; i++) {
        current[i] = torque[i]/Ke[i];
    }

    // Differentiate current and position
    determineDt();
    doubles current_dot(N,0.0);
    doubles velocity(N,0.0);
    current_dot = calculatederivative_current(current);
    velocity = calculatederivative_position(position);

    // Calculate Voltage - Eq 7- 8 from Electric Drives, An integrated Approach by Ned Mohan multiplied by Volt2PWM to convert to PWM value
    doubles voltage(N,0.0);
    for (uint i = 0; i < N; i++) {
        voltage[i] = (Ke[i]*velocity[i]/gearratio[i] + Ra[i]*current[i] + La[i]*current_dot[i])*Volt2PWM[i];
    }

    // Write Output 
    outport.write(voltage);
}

void MotorCharacteristics::determineDt()
{
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    dt = (new_time - old_time);
    old_time = new_time;
    return;
}

doubles MotorCharacteristics::calculatederivative_current(doubles diff_in_current)
{
    doubles diff_out_current(N,0.0);
    //  Determine Derivative
    if (dt > Ts*0.9 && dt < Ts*1.1) {
      for (uint i = 0; i < N; i++) {
        diff_out_current[i]  = b[0] * diff_in_current[i];
        diff_out_current[i] += b[1] * previous_input_current[i];
        diff_out_current[i] -= a[1] * previous_output_current[i];
      }
    } else {
      diff_out_current = previous_output_current;
    }

    previous_input_current  = diff_in_current;
    previous_output_current = diff_out_current;

    return diff_out_current;
}

doubles MotorCharacteristics::calculatederivative_position(doubles diff_in_position)
{
    doubles diff_out_position(N,0.0);
    //  Determine Derivative
    if (dt > Ts*0.9 && dt < Ts*1.1) {
      for (uint i = 0; i < N; i++) {
        diff_out_position[i]  = b[0] * diff_in_position[i];
        diff_out_position[i] += b[1] * previous_input_position[i];
        diff_out_position[i] -= a[1] * previous_output_position[i];
      }
    } else {
      diff_out_position = previous_output_position;
    }

    previous_input_position  = diff_in_position;
    previous_output_position = diff_out_position;

    return diff_out_position;
}

ORO_CREATE_COMPONENT(FORCECONTROL::MotorCharacteristics)
