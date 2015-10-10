#pragma once

#include "Kinect2.h"
#include <boost/thread.hpp>
#include "cinder/Vector.h"
#include <limits>
#include <algorithm>
#include "utils/MovingAverage.h"
#include "cinderopencv.h"
#include "ui/Grapher.h"
#include "utils/KalmanFilter.h"


class UserStats
{
public:
	// bounding box in world coordinates.
	ci::Vec3f lowerBound;
	ci::Vec3f centroid;
	ci::Vec3f centroidVel;
	ci::Vec3f upperBound;
	float height;

	/// quantity of motion
	float qom;
	float qomWithDecay;

	/// mean distance from centre
	float mdc;

	float timestamp;

	UserStats()
		: height(0)
		, qom(0)
		, qomWithDecay(0)
		, mdc(0)
		, timestamp(0)
	{}
};

class UserStatsCalculator
{
public:
	UserStatsCalculator();
	virtual ~UserStatsCalculator();

	// point cloud is CV_32FC3
	// userIndexMask (CV_8UC1) should be zero for any pixels not belonging to a user.
	// will assume all non-zero pixels within the bouding box belong to the user in question
	// maskedDepth8u is CV_8UC1. All matrices should be same size and will not be modified by this class
	// Thread safe
	void update(cv::Mat const& pointCloud, cv::Mat const& userIndexMask, cv::Mat const& maskedDepth8u, cv::Rect const& boundingBox, double timestamp);

	/// thread safe
	UserStats getUserStats() const;
	double getTimestamp() const;

	// Thread safe
	void draw();

private:
	void resetFilter();

	typedef boost::mutex Mutex;
	typedef boost::lock_guard<Mutex> Lock;

	// PARAMS
	bool mDrawDifferenceImage;
	// to map QOM to [0.1]
	float mQomLowerBound;
	float mQomUpperBound;
	float mQomFilterMeasurementVariance;
	float mQomFilterProcessVariance;
	bool mQomFilterHasVel;
	bool mQomFilterHasAccel;

	float mQomWithDecayDownwardFilterAlpha;

	float mMdcLowerBound;
	float mMdcUpperBound;

	// Input:
	cv::Mat mPointCloud;
	cv::Mat mUserIndexMask;
	cv::Mat mMaskedDepth8u;
	cv::Rect mBoundingBox;

	// Output
	mutable Mutex mMutex;
	// ** guards:
	cv::Mat mDifferenceImage;
	UserStats mStats;
	double mTimestamp;
	KalmanFilter mQomFilter;
	Grapher mQomGrapher;
	Grapher mQomGrapherPreFilter;
	Grapher mQomWithDecayGrapher;
	Grapher mMdcGrapher;
	// **

	// INTERNAL VARS:
	MovingAverage<cv::Mat> mMaskedDepthMovingAverage;

	void updateBounds(double dt);
	void updateQom();
	void updateMdc(); ///< mean distance from center
};
