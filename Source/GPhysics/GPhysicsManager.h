#ifndef __GPhysicsManager_H__
#define __GPhysicsManager_H__

// my own includes
#include "GRigidBody.h"
#include "GBaseSpring.h"

#include <list>
#include <iterator>
// maths libraries
#include "cwmtx/vector.h"
#include "cwmtx/quatern.h"

#include "Common.h"

class GPhysicsManager
{
public:
    GPhysicsManager();
    ~GPhysicsManager();

    void Solve( double deltaTime );

    GRigidBody* AddBody( CwMtx::CWTVector<double> position,
                         CwMtx::CWTQuaternion<double> rotation,
                         double mass,
                         double inertia );

private:
    double mTotalTime;

    std::list<GRigidBody*> mBodies;
    //std::list<GBaseSpring*> mSprings;

};

#endif
