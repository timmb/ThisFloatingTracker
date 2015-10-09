#pragma once

#include "CinderOpenCv.h"

/// Filter for float
class KalmanFilter
{
public:
	/// hasAccel only valid if hasVel==true
	KalmanFilter(bool hasVel=false, bool hasAccel=false, float initialValue = 0.f, float measurementVariance = 0.02f, float processVariance = 0.1f);
	
	void setup(bool hasVel, bool hasAccel, float initialValue = 0.f, float measurementVariance = 0.02f, float processVariance = 0.1f);
	/// calls setup and resets the filter only if these parameters (excluding initialValue) are different from the ones it already had.
	void setupIfDifferent(bool hasVel, bool hasAccel, float initialValue = 0.f, float measurementVariance = 0.02f, float processVariance = 0.1f);
	float update(float newValue);

	cv::KalmanFilter filter;

private:
	bool mHasVel;
	bool mHasAccel;
	float mMeasurementVariance;
	float mProcessVariance;
};