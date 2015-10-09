#pragma once

#include "CinderOpenCV.h"
#include <boost/thread.hpp>
#include "utils/MovingAverage.h"
#include <atomic>

class OpticalFlow
{
public:
	OpticalFlow();
	virtual ~OpticalFlow();

	// thread safe access, returns shared data but it won't be modified to by this class again
	// Optical flow with lengths tweaked, etc.
	cv::Mat getFlow() const;
	/// Optical flow direct as it comes from the algorithm
	cv::Mat getRawFlow() const;
	double getFlowTimestamp() const { return mFlowTimestamp; }

	void start();
	void stop();

	void draw();

private:
	typedef boost::shared_lock<boost::shared_mutex> SharedLock;
	typedef boost::unique_lock<boost::shared_mutex> UniqueLock;

	void threadFunction();
	void drawFlow(cv::Mat const& flowMat) const;

	std::unique_ptr<boost::thread> mThread;
	std::atomic<bool> mIsRunning;
	std::atomic<double> mFlowTimestamp;


	// PARAMETERS
	bool mDrawRawFlow;
	bool mDrawFlow;
	double mFlowDrawScale;

	// flow
	ci::Vec2i mFlowPreResize;
	int mFlowPreBlur;
	double mFlowMinLength;
	double mFlowMaxLength;
	//int mFlowFilter;

	// VARS
	cv::Mat mPrevResizedBodyIndex;
	MovingAverage<cv::Mat> mFlowMovingAverage;
	mutable boost::shared_mutex mMutex;
	// guards the following
	cv::Mat mRawFlow;
	cv::Mat mFlow;

};