#include "utils/KalmanFilter.h"

/// hasAccel only valid if hasVel==true
KalmanFilter::KalmanFilter(bool hasVel, bool hasAccel, float initialValue, float measurementVariance, float processVariance)
	: filter()
	, mHasVel(hasVel)
	, mHasAccel(hasVel && hasAccel)
	, mMeasurementVariance(measurementVariance)
	, mProcessVariance(processVariance)
{
	setup(hasVel, hasAccel, initialValue, measurementVariance, processVariance);
}

void KalmanFilter::setupIfDifferent(bool hasVel, bool hasAccel, float initialValue, float measurementVariance, float processVariance)
{
	if (hasVel != mHasVel || hasAccel != mHasAccel || measurementVariance != mMeasurementVariance || processVariance != mProcessVariance)
	{
		mHasVel = hasVel;
		mHasAccel = hasAccel;
		mMeasurementVariance = measurementVariance;
		mProcessVariance = processVariance;
		setup(hasVel, hasAccel, initialValue, measurementVariance, processVariance);
	}
}

void KalmanFilter::setup(bool hasVel, bool hasAccel, float initialValue, float measurementVariance, float processVariance)
{
	bool hasAcceleration = hasVel && hasAccel;
	filter = cv::KalmanFilter((hasAcceleration) ? 3 : hasVel ? 2 : 1, 1, 0, CV_32F);
	if (hasAcceleration)
	{
		assert(filter.transitionMatrix.size() == cv::Mat(cv::Matx33f()).size());
		filter.transitionMatrix = cv::Mat(cv::Matx33f(1.f, 1.f, 0.5f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f));
	}
	else if (hasVel)
	{
		assert(filter.transitionMatrix.size() == cv::Mat(cv::Matx22f()).size());
		filter.transitionMatrix = cv::Mat(cv::Matx22f(1.f, 1.f, 0.f, 1.f));
	}
	else
	{
		assert(filter.transitionMatrix.size() == cv::Size(1, 1));
		filter.transitionMatrix = cv::Mat(1, 1, CV_32F, cv::Scalar(1.f));
	}
	cv::setIdentity(filter.processNoiseCov, cv::Scalar::all(processVariance));
	cv::setIdentity(filter.measurementMatrix);
	cv::setIdentity(filter.measurementNoiseCov, cv::Scalar::all(measurementVariance));
	cv::setIdentity(filter.statePre, cv::Scalar(initialValue));
	cv::setIdentity(filter.statePost, cv::Scalar(initialValue));
}

float KalmanFilter::update(float newValue)
{
	// for debugging
	static bool printFilter = false;
	//if (printFilter)
	//{
	//	std::cout << "KalmanFilter:"
	//		<< "\nstatePre "<<filter.statePre
	//		<< "\nstatePost" << filter.statePost
	//		<< "\nerrorCovPre"<<filter.errorCovPre
	//		<< "\nerrorCovPost"<<filter.errorCovPost
	//		<<
	//}
	cv::Mat measurement(1, 1, CV_32F, cv::Scalar(newValue));
	filter.predict();
	cv::Mat corrected = filter.correct(measurement);
	if (corrected.rows > 0 && corrected.cols > 0 && corrected.type() == CV_32F)
	{
		return corrected.at<float>(0);
	}
	else
	{
		assert(false);
		return newValue;
	}
}