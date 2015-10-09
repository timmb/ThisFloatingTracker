#pragma once

#include <boost/thread.hpp>
#include "Kinect2.h"
#include "cinder/Vector.h"
#include "cinderOpenCv.h"
#include "utils/MovingAverage.h"
#include "cinder/PolyLine.h"
#include "input/Filter.h"
#include "input/UserStats.h"
#include "input/KinectInput.h"
#include "input/OpticalFlow.h"
#include "input/ControlPointTracker.h"
#include "input/SkeletonFinder.h"
#include "ui/KinectAligner.h"

using namespace std;
using namespace ci;

namespace cinder
{
	typedef PolyLine<Vec2i> PolyLine2i;
}



class Input
{
public:
	Input();
	virtual ~Input();
	void start();
	// blocks until threads have stopped
	void stop();

	void update();
	void draw();

	/// The area covered by the Kinect relative to Render area coordinates
	/// replacing gCameraArea and gCameraOffset.
	/// NB render coordinates have y going upwards
	ci::Rectf getKinectArea() const { return mKinectRenderArea; }
	void setKinectArea(ci::Rectf const& newKinectRenderArea) { mKinectRenderArea = newKinectRenderArea; }
	/// Size of image recieved from Kinect (kinect coordinates a.k.a. kinect pixels)
	ci::Vec2i getKinectImageSize() const { return mKinectImageSize; }
	vector<ControlPoint> getControlPoints() const { return tracker.getControlPoints(); }

	/// Debug draw depth image
	bool mDrawDepth;

	Vec2f mapKinectToRender(Vec2f const& v) const { return mKinectToRenderMapping.map(v); }
	Vec3f mapKinectToRender(Vec3f const& v) const { return Vec3f(mapKinectToRender(v.xy()), v.z); }
	Vec2f mapRenderToKinect(Vec2f const& v) const { return mRenderToKinectMapping.map(v); }
	// Maps scalar values based on the average difference in size between kinect render are and kinect input coordinate space
	float mapKinectToRender(float f) const { Vec2f v = mapKinectToRender(Vec2f(f, f)); return (v.x + v.y) / 2.f; }
	float mapKinectToWorld(float f) const { Vec3f vf = mapKinectToWorld(Vec2f(f, f)); Vec3f v0 = mapKinectToWorld(Vec2f(0, 0)); Vec2f v(vf.x - v0.x, vf.y - v0.y); return (v.x + v.y) / 2.f; }
	/// Uses the point cloud to map from kinect image space to world (Camera) coordinates.
	/// Input values outsie the kinect area region will be clamped to the border of that region
	/// This function is not thread-safe outside the main thread, as point cloud is updated in the update() function
	Vec3f mapKinectToWorld(Vec2f const& v) const;
	/// \param filterNoise will grab a neighbouring value if the calculated is (0,0,0) on the point cloud (indicates no information)
	Vec3f mapKinectToWorld(Vec2i const& v, bool filterNoise = false) const;
	/// Orthogonal projection from world coordinates to Kinect space. Based on mProjectionWallDistance
	/// to calculate the real world width of the Kinect area
	Vec2i mapWorldToKinect(Vec3f const& v) const;
	vector<Vec2i> mapWorldToKinect(vector<Vec3f> const& v) const;
	/// Orthogonal projection from world coordinates to render space. z is left unchanged
	Vec3f mapWorldToRender(Vec3f const& v) const;
	Vec2f mapWorldToRender(Vec2f const& v) const;

	/// Setup the window matrices to render the space
	/// [0,0]-size into getKinectArea().
	void setMatricesKinect(Vec2i const& size, bool originUpperLeft) const;

	/// Get distance of kinect (real world origin) from projection wall in metres
	float getProjectionWallDistance() const { return mProjectionWallDistance; }

	KinectInput kinect;
	OpticalFlow flow;
	UserStatsCalculator userStats;
	SkeletonFinder skeletonFinder;
	ControlPointTracker tracker;

	// NONPERSISTENT OPTIONS
	/// When input is paused, calculations should continue but no new data should arrive, to allow parameters to be tweaked.
	bool isInputPaused;


private:

	// 
	bool mDrawMaskedDepth8u;
	bool mDrawBodyIndex;
	bool mDrawContourAveragedBodyIndex;
	bool mDrawRawContours;
	bool mDrawContours;
	bool mDrawSmoothedContourAngles;
	bool mDrawShapes;
	//bool mDrawFilteredThinnedImage;
	
	/// The space in the render in render coordinates the the kinect image maps to.
	/// Accessed only by main thread
	ci::Rectf mKinectRenderArea;
	/// How far from the Kinect to the projection wall, in metres. Used to relate mKinectRenderArea to
	/// points in real world coordinates
	float mProjectionWallDistance;
	mutable boost::shared_mutex mLastPointCloudMutex;
	// ** guards:
	/// The most recent point cloud, for mapping points. Updated in update()
	cv::Mat mLastPointCloud;
	// **

	KinectAligner mKinectAligner;

	// contours

	int mContourBodyAveragePeriod;
	int mPreContourDilationAmount;
	double mContourSmoothEpsilon;
	double mMinContourArea;
	bool mEnableContourShapes;
	/// in radians. min angle to consider something an apex corner of a shape
	double mMinCornerAngle;
	/// max number of sub-contour shapes to find
	int mMaxShapes;
	/// in radians. min angle that when negated can indicate the end of an extracted shape (e.g. a limb)
	double mMinTerminationAngle;
	/// min extent of sub-contour (area/convex hull area)
	double mMinShapeExtent;

	// skeleton finder
	/// the biggest mMaxSkeletons contours will be sent to the skeleton tracker
	int mMaxSkeletons;
	bool mSkeletonsPreciseProcess;
	float mConnectionThreshold;
	float mSkeletonSmoothingEpsilon;


	/// input received from kinect is this size
	Vec2i mKinectImageSize;
	ci::RectMapping mKinectToRenderMapping;
	ci::RectMapping mRenderToKinectMapping;



	// CONTOURS
	void contourThreadFunction();
	/// output is put in src.shapes
	void findContourShapes(Contour& src) const;
	vector<Contour> findContours(cv::Mat const& bodyIndexImage) const;
	unique_ptr<boost::thread> mContourThread;
	std::atomic<double> mContourTimestamp;
	boost::circular_buffer<cv::Mat> mPrevBodyIndexes;
	mutable boost::mutex mContourMutex;
	// ** guards the following **
	cv::Mat mContourAveragedBodyIndex;
	vector<Contour> mSmoothedContours;
	vector<ci::PolyLine2i> mSkeletons;
	cv::Mat mThinnedImage;
	//UserStats mUserStats;
	/// Depth image masked for users downscaled to 8 bits
	cv::Mat mMaskedDepth8u;
	//cv::Mat mFilteredThinnedImage;
	// **
	/// range of values in mContours
	ci::Vec2i mContourImageSize;
	//

	bool mIsRunning;
};