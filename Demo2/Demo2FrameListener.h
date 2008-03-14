#include "../Source/GFrameListener/GFrameListener.h"

class Demo2FrameListener : public GFrameListener
{
public:
    Demo2FrameListener( RenderWindow* win,
                        Camera* cam,
                        bool bufferedKeys = false,
                        bool bufferedMouse = false,
                        bool bufferedJoy = false);

    virtual ~Demo2FrameListener();

	virtual bool frameStarted(const FrameEvent& evt);

	virtual bool frameEnded(const FrameEvent& evt);

	void setCarEntity( Entity* carChassis );
	void setSceneManager( SceneManager* sceneMgr )
	{
	    mSceneMgr = sceneMgr;
	    mRaySceneQuery = mSceneMgr->createRayQuery(Ray());
	}

	void printDebug();

private:
    Entity* mCarChassis;
    SceneManager* mSceneMgr;
    Ogre::RaySceneQuery* mRaySceneQuery;
};
