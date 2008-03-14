#ifndef __Common_Physics_H__
#define __Common_Physics_H__

namespace Integration
{
    struct State
    {
        State()
        :
        position( 3 ),
        velocity( 3 )
        {
        }

        ~State() {}

        CwMtx::CWTVector<double> position;
        CwMtx::CWTVector<double> velocity;
    };
}

#endif
