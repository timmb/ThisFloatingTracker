#pragma once
#include "Thinner.h"
#include "Contour.h"
#include "CinderOpenCv.h"
#include <boost/thread.hpp>

class SkeletonFinder
{
public:
	SkeletonFinder();
	virtual ~SkeletonFinder();

	/// May be called from any thread
	void update(cv::Mat const& bodyIndexImage, std::vector<Contour> const& contours);
	/// Thread safe
	void draw();

	/// Returns a copy of data but will no longer be modified by this class
	/// Thinned out image
	/// CV_8UC1
	cv::Mat getSkeletonImage() const;
	/// Binary image that is 255 under a contour, 0 elsewhere
	/// CV_8UC1
	cv::Mat getContourMask() const;

private:
	// PARAMS
	//bool mDrawSkeletons;
	bool mDrawThinnedImage;



	typedef boost::shared_mutex Mutex;
	typedef boost::shared_lock<Mutex> SharedLock;
	typedef boost::unique_lock<Mutex> UniqueLock;

	mutable Mutex mMutex;
	// ** guards
	cv::Mat pContourMask;
	cv::Mat pSkeletonImage;
	// **
};

