#ifndef __GRigidBody_H__
#define __GRigidBody_H__

#include "cwmtx/vector.h"
#include "cwmtx/quatern.h"

#include "Common.h"

class GRigidBody
{
public:
    GRigidBody( CwMtx::CWTVector<double> position,
                CwMtx::CWTQuaternion<double> rotation,
                double mass,
                double inertia );

    ~GRigidBody() {}

    CwMtx::CWTVector<double> GetPosition()
    {
        return mPosition;
    }

    CwMtx::CWTQuaternion<double> GetRotation()
    {
        return mRotation;
    }

    CwMtx::CWTVector<double> GetVelocity()
    {
        return mVelocity;
    }

    CwMtx::CWTVector<double> GetAngularVelocity()
    {
        return mAngularVelocity;
    }

    CwMtx::CWTVector<double> GetMomentum()
    {
        return mMomentum;
    }

    CwMtx::CWTVector<double> GetAngularMomentum()
    {
        return mAngularMomentum;
    }

    void SetPosition( CwMtx::CWTVector<double> position )
    {
        mPosition = position;
    }

    void SetRotation( CwMtx::CWTQuaternion<double> rotation )
    {
        mPosition = rotation;
    }

    CwMtx::CWTVector<double> Acceleration( const Integration::State &state, double dt, float t );
    Integration::State EvaluateFirstDerivative( const Integration::State &initial, double t );
    Integration::State EvaluateSecondDerivative( const Integration::State &initial, double t, double dt, const Integration::State &d );

private:

    // Pos and Rot
    CwMtx::CWTVector<double> mPosition;
    CwMtx::CWTQuaternion<double> mRotation;

    // Physicsy variables
    CwMtx::CWTVector<double> mVelocity;
    CwMtx::CWTVector<double> mAngularVelocity;
    CwMtx::CWTVector<double> mMomentum;
    CwMtx::CWTVector<double> mAngularMomentum;

    double mMass;
    double mIntertia;

    CwMtx::CWTVector<double> mForce;
};

#endif
