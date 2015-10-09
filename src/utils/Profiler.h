#pragma once
#include "utils/MovingAverage.h"
#include "utils/MovingPeak.h"
#include <memory>
#include <boost/thread/mutex.hpp>

#define START(eventName) gProfiler.start(#eventName);
#define END(eventName) gProfiler.end(#eventName);
/// Scoped profiling
#define PROFILE(eventName) ScopedProfile ___ScopedProfile___(#eventName);

class Profiler
{
public:
	Profiler();
	void start(std::string const& eventName);
	void end(std::string const& eventName);

	bool isEnabled;

	/// Call every frame to calculate total time spent in each event
	void update();

private:
	void checkRegistered(std::string const& eventName);

	std::set<std::string> mRegisteredEvents;

	/// event name, start time
	std::map<std::string, double> mStartTimes;
	std::map<std::string, double> mCumulativeDurationThisUpdate;

	std::map<std::string, std::shared_ptr<double>> mLastDurations;
	std::map<std::string, std::shared_ptr<double>> mAverageDurations;
	std::map<std::string, std::shared_ptr<double>> mPeakDurations;
	std::map<std::string, MovingAverage<double>> mMovingAverages;
	std::map<std::string, MovingPeak<double>> mMovingPeaks;

	boost::mutex mMutex;
};

extern Profiler gProfiler;

class ScopedProfile
{
public:
	ScopedProfile(std::string const& name_)
		: name(name_)
	{
		gProfiler.start(name);
	}

	virtual ~ScopedProfile()
	{
		gProfiler.end(name);
	}

	std::string name;
};

