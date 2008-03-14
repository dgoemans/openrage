#include "GPhysicsManager.h"

//==================================================================

GPhysicsManager::GPhysicsManager()
:
mTotalTime( 0.0 ),
mBodies( )
{

}

//==================================================================

GPhysicsManager::~GPhysicsManager()
{
    std::list<GRigidBody*>::iterator it = mBodies.begin();
    while( mBodies.end() != it++ )
    {
        delete *it;
    }
}

//==================================================================

void GPhysicsManager::Solve( double deltaTime )
{
    std::list<GRigidBody*>::iterator it = mBodies.begin();

    for( it = mBodies.begin(); it != mBodies.end(); ++it )
    {
        GRigidBody* current = *it;

        Integration::State bodyState;
        bodyState.position = current->GetPosition();
        bodyState.velocity = current->GetVelocity();

        Integration::State first = current->EvaluateFirstDerivative( bodyState, mTotalTime );
        Integration::State secondHalf1 = current->EvaluateSecondDerivative( bodyState, mTotalTime, deltaTime*0.5f, first );
        Integration::State secondHalf2 = current->EvaluateSecondDerivative( bodyState, mTotalTime, deltaTime*0.5f, secondHalf1 );
        Integration::State thirdHalf2 = current->EvaluateSecondDerivative( bodyState, mTotalTime, deltaTime, secondHalf2 );

        const CwMtx::CWTVector<double> dxdt = 1.0/6.0 * (first.position + (secondHalf1.position + secondHalf2.position)*2.0 + thirdHalf2.position);
        const CwMtx::CWTVector<double> dvdt = 1.0/6.0 * (first.velocity + (secondHalf1.velocity + secondHalf2.velocity)*2.0 + thirdHalf2.velocity);

        bodyState.position = bodyState.position + dxdt*deltaTime;
        bodyState.velocity = bodyState.velocity + dvdt*deltaTime;
    }

    mTotalTime += deltaTime;
}

//==================================================================

GRigidBody* GPhysicsManager::AddBody( CwMtx::CWTVector<double> position,
                                      CwMtx::CWTQuaternion<double> rotation,
                                      double mass,
                                      double inertia )
{
    mBodies.push_back( new GRigidBody( position, rotation, mass, inertia ) );
    return mBodies.back();
}

//==================================================================
