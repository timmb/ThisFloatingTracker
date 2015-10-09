#pragma once

#include <boost/thread.hpp>
#include "cinder/Vector.h"
#include "CinderOpenCv.h"
#include "input/Contour.h"
#include "input/ControlPoint.h"


class ControlPointTracker
{
public:
	ControlPointTracker();
	virtual ~ControlPointTracker();

	/// Safe to call from a thread
	void update(cv::Mat const& depthImage, cv::Mat const& bodyMask, cv::Mat const& skeletonImage, std::vector<Contour> const& contours);
	/// Thread safe
	void draw();

	/// Thread safe
	std::vector<ControlPoint> getControlPoints() const;

	/// Set this below 1 to pull control points towards user centroid
	float scaleFromUserCentroid;

private:
	// OPTIONS

	bool mDrawRawControlPoints;
	bool mDrawEstimatedControlPoints;
	bool mDrawRawControlPointClusters;
	bool mDrawControlPoints3d;
	bool mDrawControlPoints2d;
	bool mDrawInactiveControlPoints;
	bool mDrawUntransformedControlPoints;
	bool mDrawDynamicSpeed;
	bool mDrawDynamicAccel;
	bool mDrawKalmanSpeed;

	// PARAMS
	// control points (end points on skeleton)
	float mScaleFlowInEstimate;
	//float mControlPointAngleThreshold; ///< For angle based identifying from skeleton
	//float mControlSkeletonEpsilon; ///< For neighbour closeness based identifying from skeleton
	float mControlPointDistanceThreshold; ///< between frames to be counted as same point
	float mControlPointClusterSize; ///< max distnace between points to join cluster
	// Pruning control points - all conditions must be matched
	int mMaxNumControlPoints;
	float mControlPointVelocityPruneThreshold; ///< close points with velocity squared difference less than this may be pruned
	float mDistancePruneThreshold; ///< points closer than this may be pruned

	/// Set above 0 to increase y value of points with distance when mScaleFromUserCentroid is < 1
	float mIncreaseHeightFromDistanceAndScale;
	/// Points closer than this will have height decreased, points above will have height increased
	float mDistanceCenter;

	/// Otherwise will use the centroid of a cluster
	bool mUseClosestPointOfClusterAsObservation;
	/// If true then new position will be estimated when matching input points to control points
	bool mUseEstimatedPositions;
	

	// INTERNAL VARS

	// Stage 1: raw control points
	void findRawPoints(cv::Mat const& skeletonImage, std::vector<Contour> const& contours);
	// Output:
	// indices of these match
	std::vector<ci::Vec3f> mRawPoints;
	std::vector<ci::Vec2i> mRawPoints2d;
	/// centroid of contour used to identify each raw point
	std::vector<ci::Vec2f> mRawPointsCentroid;
	std::vector<float> mRawPointsCentroidDistanceSq;


	// Stage 2: cluster points
	void clusterRawPoints();
	// Output:
	// indices of these match:
	std::vector<ci::Vec3f> mClusteredRawPoints;
	std::vector<ci::Vec2f> mClusteredRawPoints2d;
	std::vector<ci::Vec2f> mClusteredRawPointCentroids; /// indices match mClusteredRawPoints
	std::vector<std::vector<ci::Vec2f>> mClusteredRawPoints2dIndividualPoints; /// indices match mClustseredRawPoints. centroid of each of these is the clustered point.


	// Stage 3: estimate current position of previously tracked control points
	void estimateControlPointPositions(cv::Mat const& depthImage, cv::Mat const& bodyMask);
	cv::Mat mPrevDepthImage;
	// Output:
	/// indices match mControlPoints. first is ID, second estimated position
	std::vector<std::pair<int, ci::Vec2f>> mPrevControlPoints2d;
	std::vector<std::pair<int, ci::Vec2f>> mEstimatedPrevControlPoints2d;

	// Stage 4: Match control points to new raw points
	void updateControlPoints();
	// Input and output:
	std::vector<ControlPoint> mControlPoints; ///< Non-mutex guarded copy of mControlPoints only used by updateControlPoints, prune and transform

	// Stage 5: Prune out unwanted points
	void pruneControlPoints();

	// Stage 6: Transformations based on e.g. user scale parameter
	void transformControlPoints();
	/// indices match mControlPoints
	std::vector<ci::Vec2f> mUntransformedControlPoints;

	// GUARDED AND SHARED VARS
	typedef boost::shared_mutex Mutex;
	typedef boost::shared_lock<boost::shared_mutex> SharedLock;
	typedef boost::unique_lock<boost::shared_mutex> UniqueLock;
	mutable Mutex mMutex;
	// ** guards the following
	ci::Vec2i pImageSize; ///< Size of the skeleton image input
	std::vector<ci::Vec3f> pRawControlPoints;
	/// Estimate of where last frame's control points would be now
	//vector<ci::Vec3f> mEstimatedControlPoints;
	std::vector<ci::Vec3f> pRawControlPointClusters;
	std::vector<std::pair<int, ci::Vec2f>> pPrevControlPoints2d;
	std::vector<std::pair<int, ci::Vec2f>> pEstimatedControlPoints2d;
	/// indices match pControlPoints
std::vector<ci::Vec2f> pUntransformedControlPoints;
	std::vector<ControlPoint> pControlPoints;
	// **
};