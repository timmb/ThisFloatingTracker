#include "UserStats.h"
#include "Common.h"
#include "Globals.h"
#include "cinder/Easing.h"

UserStatsCalculator::UserStatsCalculator()
: mQomGrapher("QoM", "UserStatsCalculator")
, mQomGrapherPreFilter("QoM pre-filter", "UserStatsCalculator")
, mQomWithDecayGrapher("QoM with decay", "UserStatsCalculator")
, mMdcGrapher("MDC", "UserStatsCalculator")
, mQomFilter(true, true)
{
#define PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "UserStatsCalculator" }));

	PARAM(mDrawDifferenceImage, false);
	PARAM(mMaskedDepthMovingAverage.mNumSamples, 10);
	PARAM(mQomLowerBound, 0.f);
	PARAM(mQomUpperBound, 1.f);
	PARAM(mQomWithDecayDownwardFilterAlpha, 0.05f);
	PARAM(mMdcLowerBound, 0.f);
	PARAM(mMdcUpperBound, 0.f);

#define FILTER_PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "UserStatsCalculator" }, "", [this](){ resetFilter(); }));

	FILTER_PARAM(mQomFilterMeasurementVariance, 0.02f);
	FILTER_PARAM(mQomFilterProcessVariance, 0.1f);
	FILTER_PARAM(mQomFilterHasVel, true);
	FILTER_PARAM(mQomFilterHasAccel, true);

	// User stats
#define VIEW(var) 	pars.addParam(new Parameter<decltype(var)>(&var, #var, { "UserStatsCalculator" }, "unsaved"));
	VIEW(mStats.qom);
	VIEW(mStats.lowerBound);
	VIEW(mStats.upperBound);
	VIEW(mStats.height);
	VIEW(mStats.mdc);

	resetFilter();
}


void UserStatsCalculator::resetFilter()
{
	Lock lock(mMutex);
	mQomFilter = KalmanFilter(mQomFilterHasVel, mQomFilterHasAccel, mStats.qom, mQomFilterMeasurementVariance, mQomFilterProcessVariance);
}

UserStatsCalculator::~UserStatsCalculator()
{
	Lock lock(mMutex);
}


void UserStatsCalculator::update(cv::Mat const& pointCloud, cv::Mat const& userIndexMask, cv::Mat const& maskedDepth8u, cv::Rect const& boundingBox, double timestamp)
{
	assert(pointCloud.type() == CV_32FC3);
	assert(userIndexMask.type() == CV_8UC1);

	Lock lock(mMutex);

	mPointCloud = pointCloud;
	mUserIndexMask = userIndexMask;
	mMaskedDepth8u = maskedDepth8u;
	mBoundingBox = boundingBox;

	double dt = timestamp - mTimestamp;

	float prevQomWithDecay = mStats.qomWithDecay;
	mStats = UserStats();
	mStats.qomWithDecay = prevQomWithDecay;
	updateBounds(dt);
	updateQom();
	updateMdc();
	mTimestamp = timestamp;
}


void UserStatsCalculator::updateBounds(double dt)
{
	if (mBoundingBox.width == 0 || mBoundingBox.height == 0
		|| mPointCloud.rows == 0 || mPointCloud.cols == 0
		|| mUserIndexMask.rows == 0 || mUserIndexMask.cols == 0
		|| mPointCloud.size() != mUserIndexMask.size()
		)
	{
		return;
	}
	if (!isBigEnough(mPointCloud, mBoundingBox) || !isBigEnough(mUserIndexMask, mBoundingBox))
	{
		assert(false);
		return;
	}

	for (int i = 0; i < 3; i++)
	{
		mStats.lowerBound[i] = std::numeric_limits<float>::max();
		mStats.upperBound[i] = -std::numeric_limits<float>::max();
	}
	cv::Vec3f userPixelsSum;
	int userPixelsCount = 0;
	// scan the point cloud within the bounding box to find the pixels at the limit
	cv::Mat cloudRoi = mPointCloud(mBoundingBox);
	cv::Mat userRoi = mUserIndexMask(mBoundingBox);
	auto it_cloud(cloudRoi.begin<cv::Vec3f>()), end_cloud(cloudRoi.end<cv::Vec3f>());
	auto it_user(userRoi.begin<uint8_t>()), end_user(userRoi.end<uint8_t>());
	for (; it_cloud != end_cloud; ++it_cloud, ++it_user)
	{
		assert(it_user != end_user);
		if (*it_user > 0)
		{
			for (int i = 0; i < 3; i++)
			{
				mStats.lowerBound[i] = std::min(mStats.lowerBound[i], (*it_cloud)[i]);
				mStats.upperBound[i] = std::max(mStats.upperBound[i], (*it_cloud)[i]);
			}
			userPixelsSum += *it_cloud;
			userPixelsCount++;
		}
	}
	if (userPixelsCount>0)
	{
		ci::Vec3f centroid = fromOcv(userPixelsSum) / userPixelsCount;
		if (dt > 0.0000001)
		{
			mStats.centroidVel = (centroid - mStats.centroid) / dt;
		}
		else
		{
			mStats.centroidVel = Vec3f();
		}
		mStats.centroid = centroid;
	}
	for (int i = 0; i < 3; i++)
	{
		if (mStats.lowerBound[i] > mStats.upperBound[i])
		{
			mStats.lowerBound = mStats.upperBound = ci::Vec3f();
		}
	}
	mStats.height = mStats.upperBound.y - mStats.lowerBound.y;
}


void UserStatsCalculator::updateQom()
{
	// convert to 16 bits so we can sum things together in the moving average without it saturating
	cv::Mat maskedDepth16u;
	mMaskedDepth8u.convertTo(maskedDepth16u, CV_16UC1);
	cv::Mat prev = mMaskedDepthMovingAverage.update(maskedDepth16u);
	cv::absdiff(prev, maskedDepth16u, mDifferenceImage);
	{
		cv::Mat tmp(mDifferenceImage.size(), mDifferenceImage.type());
		cv::medianBlur(mDifferenceImage, tmp, 3);
		mDifferenceImage = tmp;
	}
	// multiply number of pixels by distance squared
	float qom = cv::sum(mDifferenceImage)[0] * mStats.centroid.z * mStats.centroid.z;
	// normalise based on frame time difference
	qom /= g.dt;
	qom = (qom - mQomLowerBound) / (mQomUpperBound - mQomLowerBound);
	// fix between 0 and 1
	qom = clamp(qom, 0.f, 1.f);
	qom = ci::easeOutCubic(qom);
	mQomGrapherPreFilter.update(qom);
	float filteredQom = mQomFilter.update(qom);
	mStats.qom = filteredQom;
	mQomGrapher.update(filteredQom);
	//mDifferenceImage = prev;
	if (filteredQom < mStats.qomWithDecay)
	{
		mStats.qomWithDecay += mQomWithDecayDownwardFilterAlpha * (filteredQom - mStats.qomWithDecay);
	}
	else
	{
		mStats.qomWithDecay = filteredQom;
	}
	mQomWithDecayGrapher.update(mStats.qomWithDecay);
}


void UserStatsCalculator::updateMdc()
{
	cv::Mat pc_roi = mPointCloud(mBoundingBox);
	cv::Mat user_roi = mUserIndexMask(mBoundingBox);
	if (pc_roi.rows == 0 || pc_roi.cols == 0 || user_roi.rows == 0 || user_roi.cols == 0)
	{
		mStats.mdc = 0;
		return;
	}
	float sum = 0.f;
	int count = 0;
	cv::Vec3f centroid(mStats.centroid.x, mStats.centroid.y, mStats.centroid.z);
	auto pc_it(pc_roi.begin<cv::Vec3f>()), pc_end(pc_roi.end<cv::Vec3f>());
	auto user_it(user_roi.begin<uint8_t>()), user_end(user_roi.end<uint8_t>());
	for (; pc_it != pc_end; ++pc_it, ++user_it)
	{
		assert(user_it != user_end);
		if (*user_it > uint8_t(0))
		{
			cv::Vec3f diff = (*pc_it - centroid);
			sum += diff.dot(diff);
			count++;
		}
	}
	mStats.mdc = count == 0 ? 0.f : ci::easeInOutQuad((sqrt(sum / count) - mMdcLowerBound) / (mMdcUpperBound - mMdcLowerBound));
	mMdcGrapher.update(mStats.mdc);
}


void UserStatsCalculator::draw()
{
	gl::pushMatrices();
	{
		Lock lock(mMutex);
		gl::color(ColorA(1, .8, .8, .8));
		if (mDrawDifferenceImage && mDifferenceImage.rows && mDifferenceImage.cols)
		{
			gInput->setMatricesKinect(fromOcv(mDifferenceImage.size()), true);
			gl::draw(fromOcv(mDifferenceImage));
		}

		mQomGrapher.draw();
		mQomGrapherPreFilter.draw();
		mQomWithDecayGrapher.draw();
		mMdcGrapher.draw();
	}
	gl::popMatrices();
}

UserStats UserStatsCalculator::getUserStats() const
{
	Lock lock(mMutex);
	return mStats;
}

double UserStatsCalculator::getTimestamp() const
{
	Lock lock(mMutex);
	return mTimestamp;
}