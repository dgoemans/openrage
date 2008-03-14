#include "GRigidBody.h"

//==================================================================

GRigidBody::GRigidBody( CwMtx::CWTVector<double> position,
                        CwMtx::CWTQuaternion<double> rotation,
                        double mass,
                        double inertia )
:
// Pos and Rot
mPosition( position ),
mRotation( rotation ),

// Physicsy variables
mVelocity(3),
mAngularVelocity(3),
mMomentum(3),
mAngularMomentum(3),

mMass( mass ),
mIntertia( inertia ),

mForce(3)
{

}

//==================================================================

CwMtx::CWTVector<double> GRigidBody::Acceleration( const Integration::State &state, double dt, float t )
{
	return mForce/mMass*dt;
}

//==================================================================

Integration::State GRigidBody::EvaluateFirstDerivative( const Integration::State &initial, double t )
{
	Integration::State output;
	output.position = initial.velocity;
	output.velocity = Acceleration(initial, 0, t);
	return output;
}

//==================================================================

Integration::State GRigidBody::EvaluateSecondDerivative( const Integration::State &initial, double t, double dt, const Integration::State &d )
{
    Integration::State derivative;
	derivative.position = initial.position + d.position*dt;
	derivative.velocity = initial.velocity + d.velocity*dt;

	Integration::State output;
	output.position = derivative.velocity;
	output.velocity = Acceleration(derivative, dt, t+dt);
	return output;
}

//==================================================================
