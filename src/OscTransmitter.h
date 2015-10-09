#pragma once

#include "OscSender.h"
#include "Common.h"

class OscTransmitter
{
public:
	OscTransmitter();

	void setup();
	void update();

private:
	// PARAMS
	std::string mDestinationHost;
	int mDestinationPort;
	bool mIsEnabled;
	bool mSendQom;
	bool mSendExtents;
	bool mSendCentroid;
	bool mSendControlPoints;

	// OBJECTS
	cinder::osc::Sender mSender;

	// VARS
	std::set<int> mLastControlPointIds;
};