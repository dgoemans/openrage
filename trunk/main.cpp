// ----------------------------------------------------------------------------
// Include the main OGRE header files
// Ogre.h just expands to including lots of individual OGRE header files
// ----------------------------------------------------------------------------
#include <OGRE/Ogre.h>
// ----------------------------------------------------------------------------
// Include the OGRE example framework
// This includes the classes defined to make getting an OGRE application running
// a lot easier. It automatically sets up all the main objects and allows you to
// just override the bits you want to instead of writing it all from scratch.
// ----------------------------------------------------------------------------

#include "Source/GPhysics/GPhysicsManager.h"
#include "Source/GRunManager/GRunManager.h"
#include "Source/GFrameListener/GFrameListener.h"
#include "Source/GUtils/GMath.h"

using namespace Ogre;

// ----------------------------------------------------------------------------
// Define the application object
// This is derived from ExampleApplication which is the class OGRE provides to
// make it easier to set up OGRE without rewriting the same code all the time.
// You can override extra methods of ExampleApplication if you want to further
// specialise the setup routine, otherwise the only mandatory override is the
// 'createScene' method which is where you set up your own personal scene.
// ----------------------------------------------------------------------------
class Demo_2 : public GRunManager, public GFrameListener
{
public:
    // Basic constructor
    Demo_2()
    :
    mCarChassis( 0 ),
    mTerrain( 0 ),
    mRaySceneQuery( 0 ),
    mPhysicsManager( 0 ),
    mCarBody( 0 )
    {
        std::cout<<"New the manager\n";
        mPhysicsManager = new GPhysicsManager();
    }

    ~Demo_2()
    {
        delete mRaySceneQuery;
        mRaySceneQuery = 0;
        delete mPhysicsManager;
        mPhysicsManager = 0;
    }

protected:

    // Just override the mandatory create scene method
    void createScene(void)
    {
        // Create the SkyBox
        mSceneMgr->setSkyBox(true, "Examples/CloudyNoonSkyBox");
        mSceneMgr->setAmbientLight( ColourValue( 0.9, 0.9, 0.9 ) );

        mCarChassis = mSceneMgr->createEntity( "Chassis", "/media/Dump/David_Code/demo_2/Demo_2/Media/Chassis.mesh" );
        SceneNode *node1 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "ChassisNode", Vector3( -50, 10, 0 ) );
        node1->attachObject( mCarChassis );
        //node1->pitch( Degree( -90 ) );

        Entity *ent2 = mSceneMgr->createEntity( "Wall2", "/media/Dump/David_Code/demo_2/Demo_2/Media/Wall.mesh" );
        SceneNode *node2 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "WallNode2", Vector3( -20, -5, 0 ) );
        node2->attachObject( ent2 );
        node2->pitch( Degree( -90 ) );

        mTerrain = mSceneMgr->createEntity( "Terrain", "/media/Dump/David_Code/demo_2/Demo_2/Media/Plane.mesh" );
        SceneNode *node3 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "TerrainNode", Vector3( 0, 0, 0 ) );
        node3->attachObject( mTerrain );
        node3->pitch( Degree( -90 ) );
        node3->setScale( 100, 100, 100 );

        Ogre::Vector3 pos = mCarChassis->getParentNode()->getPosition();

        CwMtx::CWTVector<double> cwPosition(3);
        cwPosition[0] = pos.x;
        cwPosition[1] = pos.y;
        cwPosition[2] = pos.z;

        std::cout<<pos<<" \n";

        mCarBody = mPhysicsManager->AddBody( cwPosition,
                                             CwMtx::CWTQuaternion<double>(),
                                             100,
                                             100 );
    }

    virtual void updatePhysics()
    {
    }

	//===========================================================

    bool setup(void)
    {

        String pluginsPath;
        // only use plugins.cfg if not static
    #ifndef OGRE_STATIC_LIB
        pluginsPath = mResourcePath + "plugins.cfg";
    #endif

        mRoot = new Root(pluginsPath,
            mResourcePath + "ogre.cfg", mResourcePath + "Ogre.log");

        setupResources();

        bool carryOn = configure();
        if (!carryOn) return false;

        chooseSceneManager();
        createCamera();
        createViewports();
        setupWindow( GRunManager::mWindow );

        // Set default mipmap level (NB some APIs ignore this)
        TextureManager::getSingleton().setDefaultNumMipmaps(5);

        // Create any resource listeners (for loading screens)
        createResourceListener();
        // Load resources
        loadResources();

        // Create the scene
        createScene();

		mRoot->addFrameListener(this);

        //mCamera->setDirection( 0,0,0 );
        mCarChassis->getParentSceneNode()->attachObject( mCamera );


        return true;

    }

	//===========================================================

	virtual bool frameStarted( const Ogre::FrameEvent& evt )
	{
	    mPhysicsManager->Solve( evt.timeSinceLastFrame );
	    CwMtx::CWTVector<double> updatedPos = mCarBody->GetPosition();
	    Real x, y, z;
        x = updatedPos[0];
        y = updatedPos[1];
        z = updatedPos[2];
        mCarChassis->getParentNode()->setPosition( Ogre::Vector3( x, y, z ) );

        using namespace OIS;

        if( GFrameListener::mWindow->isClosed() )	return false;

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

//        if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
//            moveCamera();

        //MoveVehicle( evt.timeSinceLastFrame );
        mDebugText = "P: " + StringConverter::toString(mCamera->getDerivedPosition()) +
                " " + "O: " + StringConverter::toString(mCamera->getDerivedOrientation());
		return true;
	}

	//===========================================================

	virtual bool frameEnded( const Ogre::FrameEvent& evt )
	{
		return true;
	}

	//===========================================================

	void MoveVehicle( Real deltaTime )
	{
        if( mCarChassis )
        {
            // Make Move
            Ogre::Vector3 pos = mCarChassis->getParentNode()->getPosition();
            pos += Ogre::Vector3( 0,-9.8*deltaTime, 0 );
            Matrix3 rotMat;
            mCarChassis->getParentNode()->getOrientation().ToRotationMatrix( rotMat );
            pos += rotMat*mTranslateVector;
            mCarChassis->getParentNode()->setPosition( pos );
            mCarChassis->getParentNode()->yaw( mYaw );

            if( mSceneMgr && started )
            {
                // Keep Above Terrain
                Ogre::Vector3 rayDirection = mCarChassis->getParentNode()->getOrientation().yAxis();
                rayDirection *= 100;
                //rayStart += mCarChassis->getParentNode()->getPosition();

                Ogre::Vector3 rayOrig = mCarChassis->getParentNode()->getOrientation().yAxis();
                //rayOrig *= -100;
                rayOrig += mCarChassis->getParentNode()->getPosition();
                Ogre::Ray chassisRay = Ogre::Ray( rayOrig, rayDirection );
                mRaySceneQuery = mSceneMgr->createRayQuery( chassisRay );
                RaySceneQueryResult &result = mRaySceneQuery->execute();
                mSceneMgr->destroyQuery( mRaySceneQuery );
                //std::cout<<"Size: "<<result.size()<<"\n";
                if( !result.empty() )
                {
                    for( unsigned int i = 0; i < result.size(); i++ )
                    {
                        if( result[i].movable == mTerrain )
                        {
                            double height = result[i].distance;
                            rayDirection.normalise();
                            rayDirection *= height;
                            std::cout<<"Int Distance: "<<height<<"\n";
                            Ogre::Vector3 rayInt = rayOrig;
                            rayInt += rayDirection;
                            //rayInt -= Ogre::Vector3( 0,0,0 );
                            mCarChassis->getParentNode()->setPosition( rayInt );
                        }
                    }
                }
            }
        }
	}

private:
    Entity* mCarChassis;
    Entity* mTerrain;
    Ogre::RaySceneQuery* mRaySceneQuery;
    GPhysicsManager* mPhysicsManager;
    GRigidBody* mCarBody;

};


// ----------------------------------------------------------------------------
// Main function, just boots the application object
// ----------------------------------------------------------------------------
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
int main(int argc, char **argv)
#endif
{
    // Create application object
    Demo_2 app;

    try
    {
        app.go();
    }
    catch( Exception& e )
    {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else

        std::cerr << "An exception has occured: " << e.getFullDescription();
#endif
    }

    return 0;

}
