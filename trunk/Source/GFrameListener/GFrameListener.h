#ifndef __GFrameListener_H__
#define __GFrameListener_H__

#include "OGRE/Ogre.h"
#include "OGRE/OgreStringConverter.h"
#include "OGRE/OgreException.h"

//Use this define to signify OIS will be used as a DLL
//(so that dll import/export macros are in effect)
#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

using namespace Ogre;

class GFrameListener: public FrameListener, public WindowEventListener
{
protected:

    void updateStats(void);

public:
	// Constructor takes a RenderWindow because it uses that to determine input context
	GFrameListener();

	virtual void setupWindow( 	RenderWindow* win,
								bool bufferedKeys = false,
								bool bufferedMouse = false,
			     				bool bufferedJoy = false );
	//Adjust mouse clipping area
	virtual void windowResized(RenderWindow* rw);
	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(RenderWindow* rw);

	virtual ~GFrameListener();

	virtual bool processUnbufferedKeyInput(const FrameEvent& evt);

	bool processUnbufferedMouseInput(const FrameEvent& evt);

	void showDebugOverlay(bool show);

	// Override frameStarted event to process that (don't care about frameEnded)
	virtual bool frameStarted(const FrameEvent& evt) = 0;

	virtual bool frameEnded(const FrameEvent& evt) = 0;

protected:
	Vector3 mTranslateVector;
	Degree mYaw;
	RenderWindow* mWindow;
	bool mStatsOn;

	std::string mDebugText;

	unsigned int mNumScreenShots;
	float mMoveScale;
	Degree mRotScale;
	// just to stop toggles flipping too fast
	Real mTimeUntilNextToggle ;
	Radian mRotX, mRotY;
	TextureFilterOptions mFiltering;
	int mAniso;

	int mSceneDetailIndex ;
	Real mMoveSpeed;
	Degree mRotateSpeed;
	Overlay* mDebugOverlay;

	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	OIS::JoyStick* mJoy;

	bool started;
};

#endif

