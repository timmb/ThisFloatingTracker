#pragma once
#include "Cinder/Vector.h"

class KinectAligner
{
public:
	KinectAligner();
	virtual ~KinectAligner();

	void update();
	void draw();

private:
	bool mIsEnabled;
	bool mWasEnabled;
	bool mWasDepthPreviouslyDrawn;
	enum Mode
	{
		NONE,
		MOVING,
		ZOOMING
	};
	Mode mMode;
	/// position in render coordinates
	/// where the transformation began. if we are moving then this
	/// is relative to bottom left. if we are zooming then this is
	/// relative to the centre of the original shape.
	ci::Vec2f mTransformOrigin;
};
