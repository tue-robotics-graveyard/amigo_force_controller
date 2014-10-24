/** CurrentFFW.cpp
 *
 * @class CurrentFFW
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

#include "CurrentFFW.hpp"

using namespace std;
using namespace RTT;
using namespace FORCECONTROL;

CurrentFFW::CurrentFFW(const string& name) : 
	    TaskContext(name, PreOperational),
	    N(0), Ts(0.0)
{
    addProperty( "Ts", Ts ).doc("A double value that specifies the sampling time. Should match the sampling time of the component that triggers it (EventPort)");
    addProperty( "N", N ).doc("An unsigned integer that specifies the number of the in and outputs.");
    addProperty( "MotorVoltageConstant", Ke ).doc("A vector containing motor voltage constants");
    addProperty( "GearRatio", gearratio ).doc("A vector containing gear ratios");
    addProperty( "TerminalResistance", Ra ).doc("A vector containing terminal resistances");
    addProperty( "ArmatureWindingInductance", La ).doc("A vector containing armature winding inductances");
    addProperty( "VoltageGains", voltage_gains ).doc("A vector containing gains to multiply voltage with for example for PWM controlled motors");
}

CurrentFFW::~CurrentFFW(){}

bool CurrentFFW::configureHook()
{
    // Adding ports
    addEventPort( "in_current", inport_current );
    addEventPort( "in_position", inport_position );
    addPort( "out_voltage", outport );

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

bool CurrentFFW::startHook()
{
    // Check Ports:
    if ( !inport_current.connected() || !inport_position.connected() ) {
        log(Error)<<"CurrentFFW: One of the input ports not connected!"<<endlog();
        return false;
    }
    if ( !outport.connected() ) {
        log(Warning)<<"CurrentFFW: Output port not connected!"<<endlog();
    }

    // Check Properties
    if (N < 1 || Ts <= 0) {
        log(Error)<<"CurrentFFW: N,Ts parameters not valid!"<<endlog();
        return false;
    }

    if (Ke.size() != N || gearratio.size() != N || Ra.size() != N || La.size() != N ) {
        log(Error)<<"CurrentFFW: MotorVoltageConstant, GearRatio, TerminalResistance, ArmatureWindingInductance parameters wrongly sized!"<<endlog();
        return false;
    }
    for (uint i = 0; i < N; i++) {
        if (Ke[i] < 0.0 || gearratio[i] < 0.0 || Ra[i] < 0.0 || La[i] < 0.0 ) {
            log(Error)<<"CurrentFFW: MotorVoltageConstant, GearRatio, TerminalResistance, ArmatureWindingInductance parameters erroneus parameters!"<<endlog();
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

void CurrentFFW::updateHook()
{
    // Read inputs
    doubles input_current(N,0.0);
    doubles input_position(N,0.0);
    inport_current.read(input_current);
    inport_position.read(input_position);

    // Differentiate input_current and input_position
    determineDt();
    doubles current_dot(N,0.0);
    doubles velocity(N,0.0);
    current_dot = calculatederivative_current(input_current);
    velocity = calculatederivative_position(input_position);

    // Calculate Voltage - Eq 7- 8 from Electric Drives, An integrated Approach by Ned Mohan multiplied by voltage_gains to convert to PWM value
    doubles voltage(N,0.0);
    for (uint i = 0; i < N; i++) {
        voltage[i] = (Ke[i]*velocity[i]/gearratio[i] + Ra[i]*input_current[i] + La[i]*current_dot[i])*voltage_gains[i];
    }

    // Write Output 
    outport.write(voltage);
}

void CurrentFFW::determineDt()
{
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    dt = (new_time - old_time);
    old_time = new_time;
    return;
}

doubles CurrentFFW::calculatederivative_current(doubles diff_in)
{
    doubles diff_out(N,0.0);
    //  Determine Derivative
    if (dt > Ts*0.9 && dt < Ts*1.1) {
      for (uint i = 0; i < N; i++) {
        diff_out[i]  = b[0] * diff_in[i];
        diff_out[i] += b[1] * previous_input_current[i];
        diff_out[i] -= a[1] * previous_output_current[i];
      }
    } else {
      diff_out = previous_output_current;
    }

    previous_input_current  = diff_in;
    previous_output_current = diff_out;

    return diff_out;
}

doubles CurrentFFW::calculatederivative_position(doubles diff_in)
{
    doubles diff_out(N,0.0);
    //  Determine Derivative
    if (dt > Ts*0.9 && dt < Ts*1.1) {
      for (uint i = 0; i < N; i++) {
        diff_out[i]  = b[0] * diff_in[i];
        diff_out[i] += b[1] * previous_input_position[i];
        diff_out[i] -= a[1] * previous_output_position[i];
      }
    } else {
      diff_out = previous_output_position;
    }

    previous_input_position  = diff_in;
    previous_output_position = diff_out;

    return diff_out;
}

ORO_CREATE_COMPONENT(FORCECONTROL::CurrentFFW)
