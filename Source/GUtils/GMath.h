#ifndef __GVector3_H_
#define __GVector3_H_

#include "OGRE/Ogre.h"

class GVector3
{
public:
    GVector3()
    :
    mX( 0.0 ),
    mY( 0.0 ),
    mZ( 0.0 )
    {

    }

    GVector3( double x, double y, double z )
    :
    mX( x ),
    mY( y ),
    mZ( z )
    {

    }

    Ogre::Vector3 ToOgreVector()
    {
       return Ogre::Vector3( mX, mY, mZ );
    }

    double X()
    {
        return mX;
    }

    double Y()
    {
        return mY;
    }

    double Z()
    {
        return mZ;
    }

    double Magnitude()
    {
        return sqrt( mX*mX + mY*mY + mZ*mZ );
    }

    double MagnitudeSqr()
    {
        return ( mX*mX + mY*mY + mZ*mZ );
    }

    void Normalise()
    {
        double mag = this->Magnitude();
        mX /= mag;
        mY /= mag;
        mZ /= mag;
    }

    double DotProduct( GVector3& other )
    {
        return ( mX * other.X() + mY * other.Y() + mZ * other.Z() );
    }

private:
    double mX;
    double mY;
    double mZ;
};

#endif
