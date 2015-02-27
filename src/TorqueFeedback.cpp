 /** TorqueFeedback.cpp
 *
 * @class TorqueFeedback
 *
 * \author Max Baeten
 * \date January, 2014
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include "TorqueFeedback.hpp"

using namespace std;
using namespace RTT;
using namespace Eigen;
using namespace FORCECONTROL;

TorqueFeedback::TorqueFeedback(const string& name) : TaskContext(name, PreOperational)
{
    // Adding ports
    addEventPort( "in_u", 								inport_u );
    addPort( "in_tau", 									inport_tau );
    addPort( "in_taudot",								inport_taudot );
    addPort( "enable",									enable_inport);
    addPort( "outu", 									outport_u );
    addPort( "outt", 									outport_tau);
    addPort( "outtd", 									outport_taudot );

    // Adding properties
    addProperty( "N", 									N );
    addProperty( "BBtinvDiagonal", 						BBtinvDiagonal );
    addProperty( "DKinvDiagonal", 						DKinvDiagonal );
}

TorqueFeedback::~TorqueFeedback(){}

bool TorqueFeedback::configureHook()
{
    // declaring size of in and output doubles
    input_u.assign(N,0.0);
    input_tau.assign(N,0.0);
    input_taudot.assign(N,0.0);
    output.assign(N,0.0);
    
    // declaration of input, intermediate and output vectors
    u.resize(N);
    tau.resize(N);
    taudot.resize(N);    
    
    // declaration of matrices
	DKinv.resize(N,N);
	BBtinv.resize(N,N);
	IMinBBtinv.resize(N,N);
	
	for ( uint i = 0; i < N; i++ ) {
		for ( uint ii = 0; ii < N; ii++ ) {
			DKinv(i,ii) = 0.0;
			BBtinv(i,ii) = 0.0;
			IMinBBtinv(i,ii) = 0.0;
		}
	}
	
	cntr = 0;
	safe = false;
    
    return true;
}

bool TorqueFeedback::startHook()
{

    // port connection checks
    if ( ( !inport_u.connected() ) || ( !inport_tau.connected() ) || ( !inport_taudot.connected() ) ) {
        log(Error)<<"TorqueFeedback: One of the inports is not connected: (in_u, in_tau, in_taudot) !"<<endlog();
        return false;
    }

    if ( ( !outport_u.connected() ) || ( !outport_tau.connected() ) || ( !outport_taudot.connected() ) ) {
        log(Error)<<"TorqueFeedback: Outport not connected!"<<endlog();
        return false;
    }
    
    // check vector input of the four input matrices:  MotorInertia, ScaledMotorInertia, Damping, Stiffness
    if (BBtinvDiagonal.size() != N) {
        log(Error)<<"TorqueFeedback: One of the input vectors of the ScaledMotorInertiaInverse Matrix has a wrong size"<<endlog();
        return false;
    }
    
    // Declare input Matrices and fill in (sub/super) diagonals
    Eigen::MatrixXd D(N,N);
    Eigen::MatrixXd K(N,N);
    
	output_u.resize(N);
	output_tau.resize(N);
	output_taudot.resize(N);
	OUTU.resize(N);
	OUTTAU.resize(N);
	OUTTAUDOT.resize(N);
    
    // Filling in (Sub/Super) Diagonals
    for ( uint i = 0; i < N; i++ ) { 
        BBtinv(i,i) = BBtinvDiagonal[i];
        DKinv(i,i) = DKinvDiagonal[i];
    }
    
    // Calculate constant matrices DKinv, BBt_inv, IdentityMinBBt_inv
    Eigen::MatrixXd I;
    I.setIdentity(N,N);
    IMinBBtinv = (I - BBtinv);
    
    log(Warning)<<"TorqueFeedback:         DKinv: [" << DKinv(0,0) << ", " << DKinv(0,1) << "; " << DKinv(1,0) << ", " << DKinv(1,1) << "]!"<<endlog();
    log(Warning)<<"TorqueFeedback:        BBtinv: [" << BBtinv(0,0) << ", " << BBtinv(0,1) << "; " << BBtinv(1,0) << ", " << BBtinv(1,1) << "]!"<<endlog();
    log(Warning)<<"TorqueFeedback:    IMinBBtinv: [" << IMinBBtinv(0,0) << ", " << IMinBBtinv(0,1) << "; " << IMinBBtinv(1,0) << ", " << IMinBBtinv(1,1) << "]!"<<endlog();

	old_time = os::TimeService::Instance()->getNSecs()*1e-9;

    return true;
} 
 
void TorqueFeedback::updateHook()
{
	long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    double dt = (new_time - old_time);
    old_time = new_time;
    if (dt>0.0014 || dt<0.0006){
		log(Warning)<<"TorqueFeedback: dt is "<< dt << endlog();
	}
	
	// send control output of safe is false is received
    if (enable_inport.read(safe) == false) {
		doubles zeros(N,0.0);
		outport_u.write(zeros);
		outport_tau.write(zeros);
		outport_taudot.write(zeros);
		return;
	}

    // Read input
	inport_u.read(input_u);
    inport_tau.read(input_tau);
    inport_taudot.read(input_taudot);
    
    // Convert doubles to vectors
    for ( uint i = 0; i < N; i++ ) {
        u[i] = input_u[i];
        tau[i] = input_tau[i];
        taudot[i] = input_taudot[i];
    }
	
	// Set to zero
    for ( uint i = 0; i < N; i++ ) {
		output_u[i] = 0.0;
		output_tau[i] = 0.0;
		output_taudot[i] = 0.0;
		OUTU[i] = 0.0;
		OUTTAU[i] = 0.0;
		OUTTAUDOT[i] = 0.0;
    }
	
	// calculate
	output_u = BBtinv*u;
	output_tau = IMinBBtinv*tau;
	output_taudot = IMinBBtinv*DKinv*taudot;

	// Convert to doubles
    for ( uint i = 0; i < N; i++ ) {
		OUTU[i] = output_u[i];
		OUTTAU[i] = output_tau[i];
		OUTTAUDOT[i] = output_taudot[i];
    }
    
    // Write output
	outport_u.write(OUTU);
	outport_tau.write(OUTTAU);
	outport_taudot.write(OUTTAUDOT);

    return;
}

ORO_CREATE_COMPONENT(FORCECONTROL::TorqueFeedback)
