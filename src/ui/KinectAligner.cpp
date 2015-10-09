#include "ui/KinectAligner.h"
#include "Common.h"
#include "Globals.h"

using namespace std;
using namespace ci;

KinectAligner::KinectAligner()
	: mIsEnabled(false)
	, mWasEnabled(false)
	, mWasDepthPreviouslyDrawn(false)
	, mMode(NONE)
{
	pars.addParam(&mIsEnabled, "Align Kinect", {}, "unsaved");

}

KinectAligner::~KinectAligner()
{

}

void KinectAligner::update()
{
	if (mIsEnabled && !mWasEnabled)
	{
		g.exclusiveController = Globals::KINECT_ALIGNER;
		mWasDepthPreviouslyDrawn = gInput->mDrawDepth;
		gInput->mDrawDepth = true;
	}
	else if (mIsEnabled && g.exclusiveController != Globals::KINECT_ALIGNER)
	{
		// somthing has stolen control
		mIsEnabled = false;
	}
	if (mIsEnabled)
	{
		Mode prevMode = mMode;
		mMode = g.mouseLeft && g.shiftDown ? ZOOMING
			: g.mouseLeft && !g.shiftDown ? MOVING
			: NONE;
		Vec2f pos = mapWindowToRender(g.mouseWindowCoordinates);
		Rectf kinectArea = gInput->getKinectArea();
		if (mMode == MOVING)
		{
			if (prevMode != MOVING)
			{
				mTransformOrigin = pos - Vec2f(kinectArea.x1, kinectArea.y1);
			}
			Vec2f newOrigin = pos - mTransformOrigin;
			Vec2f size = kinectArea.getSize();
			kinectArea = Rectf(newOrigin, newOrigin + size);
			gInput->setKinectArea(kinectArea);
		}
		else if (mMode == ZOOMING)
		{
			if (prevMode != ZOOMING)
			{
				mTransformOrigin = pos - kinectArea.getCenter();
			}
			Vec2f const& prevOffsetFromCenter = mTransformOrigin;
			Vec2f newOffsetFromCenter = pos - kinectArea.getCenter();
			Vec2f scale = newOffsetFromCenter / prevOffsetFromCenter;
			mTransformOrigin = newOffsetFromCenter;
			//Vec2f scale = newScale / originalScale;
			scale.x = abs(scale.x);
			scale.y = abs(scale.y);
			kinectArea.scaleCentered(scale);
			gInput->setKinectArea(kinectArea);
		}
		if (g.keysPressed.count(app::KeyEvent::KEY_BACKSPACE))
		{
			gInput->setKinectArea(Rectf(-1, -1, 1, 1));
		}
		mWasEnabled = true;
	}
	else if (!mIsEnabled && mWasEnabled)
	{
		// if depth is still showing then set it to what it was before we were enabled
		if (gInput->mDrawDepth)
		{
			gInput->mDrawDepth = mWasDepthPreviouslyDrawn;
		}
		mWasEnabled = false;
		mMode = NONE;
	}
}


void KinectAligner::draw()
{
	if (mIsEnabled)
	{
		gl::color(ColorA::white());
		drawStringRenderSpace("Align Kinect: drag to move, shift drag to zoom, backspace to reset", Vec2f(gRenderArea.x1 + 0.2, 0));
	}
}