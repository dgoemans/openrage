 #include "Demo2FrameListener.h"

// Constructor takes a RenderWindow because it uses that to determine input context
Demo2FrameListener::Demo2FrameListener( RenderWindow* win,
                                        Camera* cam,
                                        bool bufferedKeys,
                                        bool bufferedMouse,
                                        bool bufferedJoy )
:
GFrameListener(win, cam, bufferedKeys, bufferedMouse, bufferedJoy ),
mCarChassis( 0 ),
mSceneMgr( 0 ),
mRaySceneQuery( 0 )
{

}

Demo2FrameListener::~Demo2FrameListener()
{
}


bool Demo2FrameListener::frameStarted(const FrameEvent& evt)
{
    using namespace OIS;

    if(mWindow->isClosed())	return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();
    if( mJoy ) mJoy->capture();

    bool buffJ = (mJoy) ? mJoy->buffered() : true;

    //Check if one of the devices is not buffered
    if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
    {
        // one of the input modes is immediate, so setup what is needed for immediate movement
        if (mTimeUntilNextToggle >= 0)
            mTimeUntilNextToggle -= evt.timeSinceLastFrame;

        // If this is the first frame, pick a speed
        if (evt.timeSinceLastFrame == 0)
        {
            mMoveScale = 1;
            mRotScale = 0.1;
        }
        // Otherwise scale movement units by time passed since last frame
        else
        {
            // Move about 100 units per second,
            mMoveScale = mMoveSpeed * evt.timeSinceLastFrame;
            // Take about 10 seconds for full rotation
            mRotScale = mRotateSpeed * evt.timeSinceLastFrame;
        }
        mRotX = 0;
        mRotY = 0;
        mTranslateVector = Ogre::Vector3::ZERO;
    }

    //Check to see which device is not buffered, and handle it
    if( !mKeyboard->buffered() )
        if( processUnbufferedKeyInput(evt) == false )
            return false;
    if( !mMouse->buffered() )
        if( processUnbufferedMouseInput(evt) == false )
            return false;

    if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
        moveCamera();


//    std::cout<<"FrameListening\n";

    if( mCarChassis )
    {
        // Make Move
        std::cout<<"Car Chassis Positioning\n";
        Ogre::Vector3 pos = mCarChassis->getParentNode()->getPosition();
        pos += Ogre::Vector3( 0,-9.8*evt.timeSinceLastFrame, 0 );
        Matrix3 rotMat;
        mCarChassis->getParentNode()->getOrientation().ToRotationMatrix( rotMat );
        pos += rotMat*mTranslateVector;
        mCarChassis->getParentNode()->setPosition( pos );
        mCarChassis->getParentNode()->yaw( mYaw );

        if( mSceneMgr )
        {
            // Keep Above Terrain
            Ogre::Vector3 rayDirection = mCarChassis->getParentNode()->getOrientation().yAxis();
            rayDirection *= 100;
            //rayStart += mCarChassis->getParentNode()->getPosition();

            Ogre::Vector3 rayOrig = mCarChassis->getParentNode()->getOrientation().yAxis();
            rayOrig *= -100;
            rayOrig += mCarChassis->getParentNode()->getPosition();

            Ray chassisRay = Ray( rayOrig, rayDirection );
            mRaySceneQuery->setRay( chassisRay );
            RaySceneQueryResult &result = mRaySceneQuery->execute();
            if( result.size() > 0 )
            {
                double height = result[0].distance;
                rayDirection.normalise();
                rayDirection *= height;
                rayOrig += rayDirection;
                rayOrig += Ogre::Vector3( 0,100,0 );
                mCarChassis->getParentNode()->setPosition( rayOrig );
            }
        }
    }



    return true;

}

bool Demo2FrameListener::frameEnded(const FrameEvent& evt)
{
    updateStats();
    return true;

}

void Demo2FrameListener::printDebug()
{
    std::cout<<"Jeronimo\n";
}

void Demo2FrameListener::setCarEntity( Entity* carChassis )
{
    std::cout<<"Assigning Car\n";
    if( carChassis )
    {
        std::cout<<"Existance Check\n";
        mCarChassis = carChassis;
    }
}
