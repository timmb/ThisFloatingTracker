#include "OscTransmitter.h"
#include "cinder/Cinder.h"
#include "utils/Profiler.h"

using namespace std;
using namespace ci;
using namespace osc;

OscTransmitter::OscTransmitter()
{
#define PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "OscTransmitter" }));
	//PARAM(mDestinationHost, "127.0.0.1");
	mDestinationHost = "127.0.0.1";
	PARAM(mDestinationPort, 2326);
	PARAM(mIsEnabled, false);
	PARAM(mSendCentroid, true);
	PARAM(mSendQom, true);
	PARAM(mSendExtents, true);
	PARAM(mSendControlPoints, true);
}

void OscTransmitter::setup()
{
	mSender.setup(mDestinationHost, mDestinationPort);
}

void OscTransmitter::update()
{
	PROFILE(OscTransmitter::update)

	int const userId = 0;
	UserStats userStats = gInput->userStats.getUserStats();
	if (mSendQom)
	{
		Message m;
		m.setAddress("/qom");
		// Only support one user at the moment
		m.addIntArg(userId);
		m.addFloatArg(userStats.qom);
		m.addFloatArg(userStats.qomWithDecay);
		mSender.sendMessage(m);
	}
	if (mSendExtents)
	{
		Message m;
		m.setAddress("/extents");
		m.addIntArg(userId);
		for (int i = 0; i < 3; i++)
		{
			m.addFloatArg(userStats.lowerBound[i]);
		}
		for (int i = 0; i < 3; i++)
		{
			m.addFloatArg(userStats.upperBound[i]);
		}
		mSender.sendMessage(m);
	}
	if (mSendCentroid)
	{
		Message m;
		m.setAddress("/centroid");
		m.addIntArg(userId);
		for (int i = 0; i < 3; i++)
		{
			m.addFloatArg(userStats.centroid[i]);
		}
		mSender.sendMessage(m);
	}

	set<int> currentControlPoints;
	if (mSendControlPoints)
	{
		vector<ControlPoint> controlPoints = gInput->getControlPoints();
		for (ControlPoint const& c : controlPoints)
		{
			Message m;
			bool const isNew = mLastControlPointIds.count(c.id) == 0;
			m.setAddress(isNew? "/controlPoint/start" : "/controlPoint/update");
			m.addIntArg(c.id);
			Vec3f point = gInput->mapKinectToRender(Vec3f(c.pos2d, c.pos.z));
			for (int i = 0; i < 3; i++)
			{
				m.addFloatArg(point[i]);
			}
			mSender.sendMessage(m);
			currentControlPoints.insert(c.id);
		}
	}
	for (int id : mLastControlPointIds)
	{
		if (currentControlPoints.count(id) == 0)
		{
			Message m;
			m.setAddress("/controlPoint/end");
			m.addIntArg(id);
			mSender.sendMessage(m);
		}
	}
	mLastControlPointIds = std::move(currentControlPoints);
}