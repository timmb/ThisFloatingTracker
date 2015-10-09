#include "utils/Profiler.h"
#include "Common.h"
//#include <boost/thread/lo

typedef boost::lock_guard<boost::mutex> Lock;

Profiler gProfiler;

Profiler::Profiler()
{
	pars.addParam(&isEnabled, "Enable profiler", { "Profiler" });
}

void Profiler::start(std::string const& eventName)
{
	if (!isEnabled)
	{
		return;
	}
	Lock lock(mMutex);
	checkRegistered(eventName);
	mStartTimes[eventName] = ci::app::getElapsedSeconds();
	mCumulativeDurationThisUpdate.emplace(eventName, 0.);
}

void Profiler::end(std::string const& eventName)
{
	if (!isEnabled)
	{
		return;
	}
	Lock lock(mMutex);

	if (mStartTimes.count(eventName) == 0)
	{
		return;
	}
	assert(mStartTimes.count(eventName) == 1);
	assert(mRegisteredEvents.count(eventName) == 1);
	double t = ci::app::getElapsedSeconds();
	if (mStartTimes.count(eventName) == 0)
	{
		return;
	}
	double t0 = mStartTimes[eventName];
	double duration = t - t0;
	mCumulativeDurationThisUpdate[eventName] += duration;
}

void Profiler::update()
{
	Lock lock(mMutex);

	for (auto& kv : mCumulativeDurationThisUpdate)
	{
		assert(mStartTimes.count(kv.first) == 1);
		*mLastDurations[kv.first] = kv.second;
		*mAverageDurations[kv.first] = mMovingAverages[kv.first].update(kv.second);
		*mPeakDurations[kv.first] = mMovingPeaks[kv.first].update(kv.second);
		kv.second = 0.;
	}
}

void Profiler::checkRegistered(std::string const& eventName)
{
	if (mRegisteredEvents.count(eventName)==0)	
	{
		mRegisteredEvents.insert(eventName);
		mLastDurations[eventName] = std::shared_ptr<double>(new double(-1));
		mAverageDurations[eventName] = std::shared_ptr<double>(new double(-1));
		mPeakDurations[eventName] = std::shared_ptr<double>(new double(-1));
		pars.addParam(mLastDurations[eventName].get(), string(eventName), { "Profiler" }, "unsaved readonly");
		pars.addParam(mAverageDurations[eventName].get(), string(eventName) + " average", { "Profiler" }, "unsaved readonly");
		pars.addParam(mPeakDurations[eventName].get(), string(eventName) + " peak", { "Profiler" }, "unsaved readonly");
	}
}