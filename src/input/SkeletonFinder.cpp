#include "input/SkeletonFinder.h"
#include "Common.h"

using namespace ci;
using namespace std;

SkeletonFinder::SkeletonFinder()
: pContourMask(0, 0, CV_8UC1)
, pSkeletonImage(0, 0, CV_8UC1)
{
#define PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "SkeletonFinder" }));

	PARAM(mDrawThinnedImage, false);
}

SkeletonFinder::~SkeletonFinder()
{
	UniqueLock lock(mMutex);
}

// temp
double thinnerTimeSum = 0.;
int thinnerTimeCount = 0;

void SkeletonFinder::update(cv::Mat const& bodyIndex, std::vector<Contour> const& contours)
{
	cv::Mat contoursImage(bodyIndex.size(), CV_8UC1, cv::Scalar(0));
	vector<vector<cv::Point>> allContours(contours.size(), {});
	for (int i = 0; i < contours.size(); i++)
	{
		allContours[i] = contours[i].points;
	}
	cv::drawContours(contoursImage, allContours, -1, cv::Scalar(255), CV_FILLED);
	{
		UniqueLock lock(mMutex);
		contoursImage.copyTo(pContourMask);
	}
	cv::Mat skeletonImage;
	//double t0 = ci::app::getElapsedSeconds();
	Thinner().process(contoursImage, skeletonImage);
	//double t1 = ci::app::getElapsedSeconds();
	//double t = t1 - t0;
	//thinnerTimeSum += t;
	//thinnerTimeCount++;
	//std::cout << "Thinner time: " << t << ", mean: "<<(thinnerTimeSum/thinnerTimeCount) << std::endl;
	skeletonImage.convertTo(skeletonImage, CV_8UC1);
	cv::threshold(skeletonImage, skeletonImage, 0, 255, CV_THRESH_BINARY);
	{
		UniqueLock lock(mMutex);
		pSkeletonImage = skeletonImage;
	}
}

void SkeletonFinder::draw()
{
	if (mDrawThinnedImage)
	{
		SharedLock lock(mMutex);
		gInput->setMatricesKinect(fromOcv(pSkeletonImage.size()), true);
		gl::enableAdditiveBlending();
		gl::color(ColorA(1, 0.5, 1, 1));
		gl::draw(fromOcv(pSkeletonImage));
	}

}

cv::Mat SkeletonFinder::getSkeletonImage() const
{
	SharedLock lock(mMutex);
	assert(pSkeletonImage.type() == CV_8UC1);
	return pSkeletonImage;
}

cv::Mat SkeletonFinder::getContourMask() const
{
	SharedLock lock(mMutex);
	assert(pContourMask.type() == CV_8UC1);
	return pContourMask;
}