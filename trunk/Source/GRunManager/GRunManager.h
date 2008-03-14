
#ifndef __GRunManager_H__
#define __GRunManager_H__

#include "OGRE/Ogre.h"
#include "OGRE/OgreConfigFile.h"

using namespace Ogre;

/** Base class which manages the standard startup of an Ogre application.
    Designed to be subclassed for specific examples if required.
*/
class GRunManager
{
public:
    /// Standard constructor
    GRunManager();
    /// Standard destructor
    virtual ~GRunManager();
    /// Start running
    virtual void go(void);

    virtual void updatePhysics() = 0;

protected:
    Root *mRoot;
    Camera* mCamera;
    SceneManager* mSceneMgr;
    RenderWindow* mWindow;
	Ogre::String mResourcePath;

    // These internal methods package up the stages in the startup process
    /** Sets up the application - returns false if the user chooses to abandon configuration. */
    virtual bool setup(void) = 0;
    /** Configures the application - returns false if the user chooses to abandon configuration. */
    virtual bool configure(void);
    virtual void chooseSceneManager(void);
    virtual void createCamera(void);
    virtual void createScene(void) = 0;    // pure virtual - this has to be overridden

    virtual void destroyScene(void){}    // Optional to override this

    virtual void createViewports(void);
    /// Method which will define the source of resources (other than current folder)
    virtual void setupResources(void);

	/// Optional override method where you can create resource listeners (e.g. for loading screens)
	virtual void createResourceListener(void);

	/// Optional override method where you can perform resource group loading
	/// Must at least do ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	virtual void loadResources(void);

};


#endif
