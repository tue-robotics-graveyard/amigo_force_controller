 /** UPF_TorqueFeedbackController.cpp
 *
 * @class TorqueFeedbackController
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
#include "UPF_TorqueFeedbackController.hpp"

using namespace std;
using namespace RTT;
using namespace Eigen;
using namespace UPF;

TorqueFeedbackController::TorqueFeedbackController(const string& name) : TaskContext(name, PreOperational)
{
    // Adding ports
    addEventPort( "in_u", inport_u );
    addEventPort( "in_tau", inport_tau );
    addEventPort( "in_taudot", inport_taudot );
    addPort( "out", outport );

    // Adding properties
    addProperty( "N", 									N );
    addProperty( "MotorInertiaSuperDiagonal", 			MotorInertiaSuperDiagonal );
    addProperty( "MotorInertiaDiagonal", 				MotorInertiaDiagonal );
    addProperty( "MotorInertiaSubDiagonal", 			MotorInertiaSubDiagonal );
    addProperty( "ScaledMotorInertiaSuperDiagonal", 	ScaledMotorInertiaSuperDiagonal );
    addProperty( "ScaledMotorInertiaDiagonal", 			ScaledMotorInertiaDiagonal );
    addProperty( "ScaledMotorInertiaSubDiagonal", 		ScaledMotorInertiaSubDiagonal );
    addProperty( "DampingSuperDiagonal", 				DampingSuperDiagonal );
    addProperty( "DampingDiagonal", 					DampingDiagonal );
    addProperty( "DampingSubDiagonal", 					DampingSubDiagonal );
    addProperty( "StiffnessSuperDiagonal", 				StiffnessSuperDiagonal );
    addProperty( "StiffnessDiagonal", 					StiffnessDiagonal );
    addProperty( "StiffnessSubDiagonal", 				StiffnessSubDiagonal );
}

TorqueFeedbackController::~TorqueFeedbackController(){}

bool TorqueFeedbackController::configureHook()
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
    Tau_m.resize(N);
    
    // declaration of matrices
	DKinv.resize(N,N);
	BBtinv.resize(N,N);
	IMinBBtinv.resize(N,N);
    
    return true;
}

bool TorqueFeedbackController::startHook()
{
    // port connection checks
    if ( ( !inport_u.connected() ) || ( !inport_tau.connected() ) || ( !inport_taudot.connected() ) ) {
        log(Error)<<"TorqueFeedbackController: One of the inports is not connected: (in_u, in_tau, in_taudot) !"<<endlog();
        return false;
    }

    if ( !outport.connected() ) {
        log(Error)<<"TorqueFeedbackController: Outport not connected!"<<endlog();
        return false;
    }

    // check vector input of the four input matrices:  MotorInertia, ScaledMotorInertia, Damping, Stiffness
    if (( MotorInertiaSuperDiagonal.size() != N ) || ( MotorInertiaDiagonal.size() != ( N - 1 ) ) || ( MotorInertiaSubDiagonal.size() != ( N - 1 )  ) ) {
        log(Error)<<"TorqueFeedbackController: One of the input vectors of the MotorInertia Matrix has a wrong size"<<endlog();
        return false;
    }

    if (( ScaledMotorInertiaSuperDiagonal.size() != N ) || ( ScaledMotorInertiaDiagonal.size() != ( N - 1 ) ) || ( ScaledMotorInertiaSubDiagonal.size() != ( N - 1 )  ) ) {
        log(Error)<<"TorqueFeedbackController: One of the input vectors of the ScaledMotorInertia Matrix has a wrong size"<<endlog();
        return false;
    }

    if (( DampingSuperDiagonal.size() != N ) || ( DampingDiagonal.size() != ( N - 1 ) ) || ( DampingSubDiagonal.size() != ( N - 1 )  ) ) {
        log(Error)<<"TorqueFeedbackController: One of the input vectors of the Damping Matrix has a wrong size"<<endlog();
        return false;
    }

    if (( StiffnessSuperDiagonal.size() != N ) || ( StiffnessDiagonal.size() != ( N - 1 ) ) || ( StiffnessSubDiagonal.size() != ( N - 1 )  ) ) {
        log(Error)<<"TorqueFeedbackController: One of the input vectors of the Stiffness Matrix has a wrong size"<<endlog();
        return false;
    }

    // Declare input Matrices and fill in (sub/super) diagonals
    Eigen::MatrixXd B, Bt, D, K;
    
    // Filling in (Sub/Super) Diagonals
    for ( uint i = 0; i < N; i++ ) { 
        B(i,i) = MotorInertiaDiagonal[i];
        Bt(i,i) = ScaledMotorInertiaDiagonal[i];
        D(i,i) = DampingDiagonal[i];
        K(i,i) = StiffnessDiagonal[i];
    }
    for ( uint i = 0; i < ( N - 1 ) ; i++ ) {
        B(i,(i+1)) = MotorInertiaSuperDiagonal[i];
        Bt(i,(i+1)) = ScaledMotorInertiaSuperDiagonal[i];
        D(i,(i+1)) = DampingSuperDiagonal[i];		
        K(i,(i+1)) = StiffnessSuperDiagonal[i];
        B((i+1),i) = MotorInertiaSubDiagonal[i];
        Bt((i+1),i) = ScaledMotorInertiaSubDiagonal[i];
        D((i+1),i) = DampingSubDiagonal[i];
        K((i+1),i) = StiffnessSubDiagonal[i];
    }

    // Calculate constant matrices DKinv, BBt_inv, IdentityMinBBt_inv
    Eigen::MatrixXd I;
    I.setIdentity(N,N);
    DKinv = D * K.inverse();
    BBtinv = B * Bt.inverse();
    IMinBBtinv = (I - (BBtinv));

	log(Warning)<<"TorqueFeedbackController: configureHook() executed properly!"<<endlog();
	
    return true;
} 
 
void TorqueFeedbackController::updateHook()
{
    // Read input doubles
    if ( !inport_u.read(input_u) == NewData ) {
        log(Error)<<"TorqueFeedbackController: updateHook() was triggered while no newdata on inputPort! This should be impossible for eventtriggered component"<<endlog();
        return;
    }
    inport_tau.read(input_tau);
    inport_taudot.read(input_taudot);

    // Convert doubles to vectors
    for ( uint i = 0; i < N; i++ ) {
        u[i] = input_u[i];
        tau[i] = input_tau[i];
        taudot[i] = input_taudot[i];
    }

    // Calculate Tau_m
    Tau_m = BBtinv*u + IMinBBtinv * (tau + DKinv*taudot);

    // Convert vector to doubles
    for ( uint i = 0; i < N; i++ ) {
        output[i] = Tau_m[i];
    }

    // Write output doubles
    outport.write(output);
}

ORO_CREATE_COMPONENT(UPF::TorqueFeedbackController)