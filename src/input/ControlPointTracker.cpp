#include "input/ControlPointTracker.h"
#include "Common.h"
#include <boost/iterator/counting_iterator.hpp>
#include <numeric>
#include "cinder/app/App.h"
#include "Globals.h"


using namespace std;
using namespace ci;


// ----------------------------------------------


ControlPointTracker::ControlPointTracker()
{
#define PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "ControlPoints" }));
#define PARAM_P(var, init) 	*var = init; pars.addParam(new Parameter<std::decay<decltype(*var)>::type>(var, #var, { "ControlPoints" }));

	PARAM(mDrawRawControlPoints, false);
	PARAM(mDrawEstimatedControlPoints, false);
	PARAM(mDrawRawControlPointClusters, false);
	PARAM(mDrawControlPoints3d, false);
	PARAM(mDrawInactiveControlPoints, false);
	PARAM(mDrawControlPoints2d, false);
	PARAM(mDrawUntransformedControlPoints, false);
	PARAM(mDrawDynamicSpeed, false);
	PARAM(mDrawDynamicAccel, false);
	PARAM(mDrawKalmanSpeed, false);


	//PARAM(mControlPointAngleThreshold, 0.1f);
	//PARAM(mControlSkeletonEpsilon, 0.1f);
	PARAM(mScaleFlowInEstimate, 1.f);
	PARAM(mControlPointClusterSize, 8.f);
	PARAM(mControlPointDistanceThreshold, 1.0);
	PARAM(ControlPoint::sPersistence, 6);
	PARAM(ControlPoint::sReluctance, 2);
	PARAM(mControlPointVelocityPruneThreshold, 0.f);
	PARAM(mDistancePruneThreshold, 0.f);
	PARAM(mMaxNumControlPoints, 5);
	PARAM(scaleFromUserCentroid, 1.f);
	PARAM(mIncreaseHeightFromDistanceAndScale, 0.f);
	PARAM(mDistanceCenter, 3.f);
	PARAM(mUseClosestPointOfClusterAsObservation, false);
	PARAM(mUseEstimatedPositions, true);


	PARAM(ControlPoint::sFilterAlpha, 0.4f);
	PARAM(ControlPoint::sFilterBeta, 0.5f);
	PARAM(ControlPoint::sTrendDecay, 0.f);
	PARAM(ControlPoint::sEnableFilter, true);
	PARAM(ControlPoint::sProcessNoisePos, ControlPoint::sProcessNoisePos);
	PARAM(ControlPoint::sProcessNoiseVel, ControlPoint::sProcessNoiseVel);
	PARAM(ControlPoint::sProcessNoiseAcc, ControlPoint::sProcessNoiseAcc);
	PARAM(ControlPoint::sMeasurementNoise, ControlPoint::sMeasurementNoise);

	PARAM_P(ControlPoint::sDynamicControlInputDecay.get(), 0.9889333f);
	PARAM_P(ControlPoint::sDynamicControlPeakDecay.get(), 0.99883333f);
	PARAM_P(ControlPoint::sDynamicControlWindowSize.get(), 5);
	PARAM_P(ControlPoint::sDynamicControlScaleSpeed.get(), 1.f);
	PARAM_P(ControlPoint::sDynamicControlScaleAccel.get(), 1.f);

	PARAM(ControlPoint::sScaleKalmanSpeed, 1.f);
}

ControlPointTracker::~ControlPointTracker()
{
	UniqueLock lock(mMutex);
}


vector<ControlPoint> ControlPointTracker::getControlPoints() const
{
	SharedLock lock(mMutex);
	return pControlPoints;
}


void ControlPointTracker::findRawPoints(cv::Mat const& skeletonImage, vector<Contour> const& contours)
{
	mRawPoints.clear();
	mRawPoints2d.clear();
	mRawPointsCentroid.clear();
	mRawPointsCentroidDistanceSq.clear();

	// Method 4 - analyse the skeleton image for pixels with only one neighbour
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect bb = contours[i].boundingBox;
		bb.x = max(bb.x - 1, 0);
		bb.width = min(bb.width + 3, skeletonImage.cols - bb.x);
		bb.y = max(bb.y - 1, 0);
		bb.height = min(bb.height + 3, skeletonImage.rows - bb.y);
		if (bb.width < 3 || bb.height < 3)
		{
			continue;
		}
		Vec2f const& centroid = contours[i].centroid;
		cv::Mat roi = skeletonImage(bb);
		assert(roi.type() == CV_8UC1);


		for (int x = 1; x < roi.cols - 1; x++)
		{
			for (int y = 1; y < roi.rows - 1; y++)
			{
				if (roi.at<uchar>(cv::Point(x, y)))
				{
					// iterate through neighbours clockwise
					static cv::Point const neighborOffsets[8] = {
						cv::Point(-1, -1),
						cv::Point(+0, -1),
						cv::Point(+1, -1),
						cv::Point(+1, +0),
						cv::Point(+1, +1),
						cv::Point(+0, +1),
						cv::Point(-1, +1),
						cv::Point(-1, +0),
					};
					//// array of zero neighbors
					//bool nonZeroNeighbors[8];
					// count number of zero-valued neighbours while also keeping track of them in array
					int nonZeroCount = 0;
					cv::Point center(x, y);
					for (int i = 0; i < 8; i++)
					{
						nonZeroCount += /*nonZeroNeighbors[i] =*/ roi.at<uchar>(center + neighborOffsets[i]) != 0u;
					}
					if (
						// if 1 non-zero neighbour then we're definitely an end point
						(nonZeroCount == 1)
						////// if 2 non-zero neighbours then consider ourselves an endpoint if both are adjacent
						//|| (nonZeroCount == 2 && (
						//(nonZeroNeighbors[0] && nonZeroNeighbors[1])
						//|| (nonZeroNeighbors[1] && nonZeroNeighbors[2])
						//|| (nonZeroNeighbors[2] && nonZeroNeighbors[3])
						//|| (nonZeroNeighbors[3] && nonZeroNeighbors[4])
						//|| (nonZeroNeighbors[4] && nonZeroNeighbors[5])
						//|| (nonZeroNeighbors[5] && nonZeroNeighbors[6])
						//|| (nonZeroNeighbors[6] && nonZeroNeighbors[7])
						//|| (nonZeroNeighbors[7] && nonZeroNeighbors[0])
						//))
						)
					{
						//// the actual end point (center) tends to be noisy, so use the non-zero value next it
						//cv::Point nonZeroNeighbor(-1, -1);
						//for (int i = 0; i < 8; i++)
						//{
						//	cv::Point p = center + neighborOffsets[i];
						//	if (roi.at<uchar>(p) != 0u)
						//	{
						//		nonZeroNeighbor = p;
						//	}
						//}
						//assert(nonZeroNeighbor != cv::Point(-1, -1));
						Vec2i point2d = Vec2i(x + bb.x, y + bb.y);
						Vec3f point3d = gInput->mapKinectToWorld(point2d, true);
						// skip any (0,0,0) points. they cause problems
						if (point3d.z > 0.001)
						{
							mRawPoints2d.push_back(point2d);
							mRawPoints.push_back(gInput->mapKinectToWorld(point2d, true));
							mRawPointsCentroid.push_back(centroid);
							mRawPointsCentroidDistanceSq.push_back(centroid.distance(point2d));
						}
					}
				}
			}
		}
	}
	assert(mRawPoints.size() == mRawPoints2d.size());
	assert(mRawPoints.size() == mRawPointsCentroid.size());
	assert(mRawPoints.size() == mRawPointsCentroidDistanceSq.size());
}


void ControlPointTracker::clusterRawPoints()
{
	// Group points that are close together. Start groups based on the furthest from the centroid
	// keep track of the centroids as we go on as we'll save that with the point later
	vector<vector<Vec3f>> pointClusters;
	// use 3d points for cluster calculations, but retain 2d point information too
	vector<vector<Vec2f>> pointClusters2d;
	mClusteredRawPointCentroids.clear();
	//sort based on distance from centroid. Furthest elements at the front.
	vector<size_t> indices(boost::counting_iterator<size_t>(0u), boost::counting_iterator<size_t>(mRawPoints.size()));
	sort(begin(indices), end(indices), [&](size_t lhs, size_t rhs) {
		return mRawPointsCentroidDistanceSq[lhs] > mRawPointsCentroidDistanceSq[rhs];
	});
	for (size_t i : indices)
	{
		Vec3f const& point = mRawPoints[i];
		Vec2f const& point2d = mRawPoints2d[i];
		auto it = begin(pointClusters);
		auto it2d = begin(pointClusters2d);
		for (; it != end(pointClusters); ++it, ++it2d)
		{
			assert(it2d != end(pointClusters2d));
			vector<Vec3f> & cluster = *it;
			vector<Vec2f> & cluster2d = *it2d;
			// new point has to be less than threshold to all points in cluster to join it
			auto jt = begin(cluster);
			for (; jt != end(cluster); ++jt)
			{
				if (point.distanceSquared(*jt) > mControlPointClusterSize)
				{
					break;
				}
			}
			// if we reached the end of the loop then this cluster is ok to put our new point in
			if (jt == end(cluster))
			{
				cluster.push_back(point);
				cluster2d.push_back(point2d);
				break; // break from searching the clusters to find a home for this point
			}
		}
		// if we reached the end of outer loop then no home was found for the cluster
		if (it == end(pointClusters))
		{
			pointClusters.push_back({ point });
			pointClusters2d.push_back({ point2d });
			mClusteredRawPointCentroids.push_back(mRawPointsCentroid[i]);
		}
	}
	// Find mean of point clusters
	mClusteredRawPoints.resize(pointClusters.size());
	mClusteredRawPoints2d.resize(pointClusters.size());
	mClusteredRawPoints2dIndividualPoints.resize(pointClusters.size());
	auto it(cbegin(pointClusters)), it_end(cend(pointClusters));
	auto it2d(cbegin(pointClusters2d)), it2d_end(cend(pointClusters2d));
	auto out(begin(mClusteredRawPoints)), out_end(end(mClusteredRawPoints));
	auto out2d(begin(mClusteredRawPoints2d)), out2d_end(end(mClusteredRawPoints2d));
	auto out2dPoints(begin(mClusteredRawPoints2dIndividualPoints)), out2dPoints_end(end(mClusteredRawPoints2dIndividualPoints));
	for (; it != it_end; ++it, ++it2d, ++out, ++out2d, ++out2dPoints)
	{
		assert(it2d != it2d_end);
		assert(out != out_end);
		assert(out2d != out2d_end);
		assert(out2dPoints != out2dPoints_end);
		vector<Vec3f> const& cluster = *it;
		vector<Vec2f> const& cluster2d = *it2d;
		assert(!cluster.empty());
		assert(!cluster2d.empty());
		assert(cluster.size() == cluster2d.size());
		*out = std::accumulate(begin(cluster) + 1, end(cluster), cluster.front()) / cluster.size();
		*out2d = std::accumulate(begin(cluster2d) + 1, end(cluster2d), cluster2d.front()) / cluster2d.size();
		// move the individual points into a member variable
		*out2dPoints = std::move(*it2d);
	}
	assert(out2dPoints == out2dPoints_end);
	assert(mClusteredRawPoints.size() == mClusteredRawPointCentroids.size());
	assert(mClusteredRawPoints2d.size() == mClusteredRawPoints.size());
	assert(mClusteredRawPoints2dIndividualPoints.size() == mClusteredRawPoints.size());
}


//vector<function<void()>> deferredDrawCalls;

void ControlPointTracker::estimateControlPointPositions(cv::Mat const& depthImage16u, cv::Mat const& bodyMask)
{
	assert(mUntransformedControlPoints.size() == mControlPoints.size());

	cv::Mat depthImage;
	depthImage16u.convertTo(depthImage, CV_8UC1);
	depthImage.copyTo(depthImage, bodyMask);
	if (mControlPoints.empty())
	{
		mEstimatedPrevControlPoints2d.clear();
	}
	else
	{
		if (mPrevDepthImage.size() != depthImage.size() || mPrevDepthImage.type() != depthImage.type())
		{
			mPrevDepthImage = depthImage;
		}
		vector<cv::Vec2f> points(mControlPoints.size());
		vector<cv::Vec2f> flowEstimate(mControlPoints.size());
		for (int i = 0; i < mControlPoints.size(); i++)
		{
			points[i] = toOcv(mUntransformedControlPoints[i]);
			//flowEstimate[i] = toOcv(mControlPoints[i].predictPos2d() - mControlPoints[i].pos2d);
		}
		vector<cv::Vec2f> flow(flowEstimate.begin(), flowEstimate.end());
		vector<uchar> statuses(mControlPoints.size());
		//vector<float> errors(mControlPoints.size());
		//cout << "\n";
		//cv::calcOpticalFlowPyrLK(mPrevDepthImage, depthImage, points, flow, statuses, errors, cv::Size(9, 9), 6, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
		// alternative
		{
			cv::Mat flowMat = gInput->flow.getRawFlow();
			Vec2f depthToFlow(flowMat.cols / (float)depthImage.cols, flowMat.rows / (float)depthImage.rows);
			Vec2f flowToDepth(depthImage.cols / (float)flowMat.cols, depthImage.rows / (float)depthImage.rows);
			for (int i = 0; i < mControlPoints.size(); i++)
			{
				Vec2f p_flow = mUntransformedControlPoints[i] * depthToFlow;
				cv::Point p((int)p_flow.x, (int)p_flow.y);
				p.x = clamp(p.x, 0, flowMat.cols - 1);
				p.y = clamp(p.y, 0, flowMat.rows - 1);
				flow[i] = toOcv(fromOcv(flowMat.at<cv::Vec2f>(p)) * flowToDepth * mScaleFlowInEstimate);
				statuses[i] = 1;
			}
		}

		mEstimatedPrevControlPoints2d.resize(mControlPoints.size());
		mPrevControlPoints2d.resize(mControlPoints.size());
		{
			auto it(mControlPoints.cbegin()), it_end(mControlPoints.cend());
			auto ut(mUntransformedControlPoints.cbegin()), ut_end(mUntransformedControlPoints.cend());
			auto et(flowEstimate.cbegin()), et_end(flowEstimate.cend());
			auto ft(flow.cbegin()), ft_end(flow.cend());
			auto st(statuses.cbegin()), st_end(statuses.cend());
			auto out(mEstimatedPrevControlPoints2d.begin()), out_end(mEstimatedPrevControlPoints2d.end());
			auto prev(mPrevControlPoints2d.begin()), prev_end(mPrevControlPoints2d.end());
			for (; it != it_end; ++it, ++ut, ++out, ++et, ++ft, ++st, ++prev)
			{
				assert(ut != ut_end);
				assert(out != out_end);
				assert(et != et_end);
				assert(ft != ft_end);
				assert(st != st_end);
				assert(prev != prev_end);
				*out = make_pair(it->id, *ut + (mUseEstimatedPositions? fromOcv(*st > 0 ? *ft : *et) : Vec2f()));
				*prev = make_pair(it->id, *ut);
			}
			assert(out == out_end && ut == ut_end && et == et_end && ft == ft_end && st == st_end && prev == prev_end);
		}
	}
	mPrevDepthImage = depthImage;
}


void ControlPointTracker::updateControlPoints()
{
	// pair up control points to raw points so that the nearest ones are paired up first
	// first is index of raw point from mClusteredRawPoints2d. second is index of control point
	list<pair<size_t, size_t>> pairs;
	for (size_t i = 0; i < mClusteredRawPoints2d.size(); i++)
	{
		for (size_t j = 0; j < mControlPoints.size(); j++)
		{
			pairs.push_back(make_pair(i, j));
		}
	}
	// indices match pairs. distance squared between raw point and control point (min of current control point pos and estimated pos)
	vector<float> pairDistances;
	// create list of esimated positions for control points
	// indices match mControlPoints
	vector<Vec2f> controlPointEstimatedPositions;
	{
		{
			controlPointEstimatedPositions.reserve(mControlPoints.size());
			auto c_it(cbegin(mControlPoints)), c_end(cend(mControlPoints));
			auto u_it(cbegin(mUntransformedControlPoints)), u_end(cend(mUntransformedControlPoints));
			for (; c_it != c_end; ++c_it, ++u_it)
			//for (ControlPoint const& c : mControlPoints)
			{
				assert(u_it != u_end);
				ControlPoint const& c = *c_it;
				controlPointEstimatedPositions.push_back(*u_it);
				Vec2f & estPos = controlPointEstimatedPositions.back();
				for (auto const& id_pos : mEstimatedPrevControlPoints2d)
				{
					if (id_pos.first == c.id)
					{
						estPos = id_pos.second;
						break;
					}
				}
			}
			assert(u_it == u_end);
			assert(controlPointEstimatedPositions.size() == mControlPoints.size());
		}

		// find distance for each pair and put in pairDistances
		{
			pairDistances.reserve(pairs.size());
			auto it_pair(begin(pairs)), end_pair(end(pairs));
			for (; it_pair != end_pair; ++it_pair)
			{
				Vec2f const& raw = mClusteredRawPoints2d[it_pair->first];
				//ControlPoint const& c = mControlPoints[it_pair->second];
				Vec2f const& pos = mUntransformedControlPoints[it_pair->second];
				Vec2f const& estPos = controlPointEstimatedPositions[it_pair->second];
				pairDistances.push_back(min(raw.distanceSquared(/*c.pos2d*/ pos), raw.distanceSquared(estPos)));
			}
		}
	}

	// now update based on the closest distance until any of:
	// - the distnace threshold is too high (break)
	// - we've run out of control points
	// - we've run out of raw points

	assert(mClusteredRawPoints2d.size() == mClusteredRawPointCentroids.size());
	// indices of mClusteredRawPoints
	set<size_t> unmatchedRawPoints;
	for (size_t i = 0; i < mClusteredRawPoints2d.size(); i++)
	{
		unmatchedRawPoints.insert(i);
	}
	// indices of mControlPoints;
	set<size_t> unmatchedControlPoints;
	for (size_t i = 0; i < mControlPoints.size(); i++)
	{
		unmatchedControlPoints.insert(i);
	}
	while (!unmatchedRawPoints.empty() && !unmatchedControlPoints.empty())
	{
		auto it_minPair = end(pairs);
		float minDistance = numeric_limits<float>::max();
		{
			auto it_pair = begin(pairs);
			auto it_dist = begin(pairDistances);
			for (; it_pair != end(pairs); ++it_pair, ++it_dist)
			{
				assert(it_dist != end(pairDistances));
				if (*it_dist < minDistance)
				{
					minDistance = *it_dist;
					it_minPair = it_pair;
				}
			}
		}
		auto minPair = *it_minPair;
		assert(it_minPair != end(pairs));

		if (minDistance > mControlPointDistanceThreshold)
		{
			break;
		}

		// update - use the closest point within the cluster as the observation OR the centroid of the cluster, depending on a param
		{
			Vec2f const* observation = nullptr;
			if (mUseClosestPointOfClusterAsObservation)
			{
				vector<Vec2f> const& individualPoints = mClusteredRawPoints2dIndividualPoints[minPair.first];
				Vec2f const& estimatedPosition = controlPointEstimatedPositions[minPair.second];
				assert(!individualPoints.empty());
				observation = &individualPoints.front();
				float distSq = observation->distanceSquared(estimatedPosition);
				for (auto it = begin(individualPoints) + 1; it != end(individualPoints); ++it)
				{
					float d = it->distanceSquared(estimatedPosition);
					if (d < distSq)
					{
						observation = &*it;
						distSq = d;
					}
				}
			}
			else
			{
				observation = &mClusteredRawPoints2d[minPair.first];
			}
			mControlPoints[minPair.second].updateWithObservation(*observation, mClusteredRawPointCentroids[minPair.first]);
		}
		// remove from sets
		assert(unmatchedRawPoints.count(it_minPair->first) == 1);
		unmatchedRawPoints.erase(minPair.first);
		assert(unmatchedControlPoints.count(it_minPair->second) == 1);
		unmatchedControlPoints.erase(minPair.second);
		// remove any pair distances that are now obsolete because they refer to an already matched element
		{
			auto it_pair(begin(pairs));
			auto it_dist(begin(pairDistances));
			while (it_pair != end(pairs))
			{
				assert(it_dist != end(pairDistances));
				if (it_pair->first == minPair.first || it_pair->second == minPair.second)
				{
					it_pair = pairs.erase(it_pair);
					it_dist = pairDistances.erase(it_dist);
				}
				else
				{
					++it_pair;
					++it_dist;
				}
			}
		}
	}

	// Update unmatched control points
	for (size_t i : unmatchedControlPoints)
	{
		assert(i < mControlPoints.size());
		mControlPoints[i].updateWithoutObservation();
	}

	// check if we need to delete any points
	for (auto it = begin(mControlPoints); it != end(mControlPoints);)
	{
		if (it->isReadyForDeletion())
		{
			it = mControlPoints.erase(it);
		}
		else
		{
			++it;
		}
	}

	// add new points for the unmatched raw points
	{
		mControlPoints.reserve(mControlPoints.size() + unmatchedRawPoints.size());
		for (size_t i : unmatchedRawPoints)
		{
			mControlPoints.emplace_back(mClusteredRawPoints2d[i], mClusteredRawPoints[i], mClusteredRawPointCentroids[i]);
		}
	}

	// update grapher variables (for debug drawing)
	for (ControlPoint & p : mControlPoints)
	{
		p.dynamicAccelGrapher.isEnabled = mDrawDynamicAccel;
		p.dynamicSpeedGrapher.isEnabled = mDrawDynamicSpeed;
		p.kalmanSpeedGrapher.isEnabled = mDrawKalmanSpeed;
	}
}

void ControlPointTracker::pruneControlPoints()
{
	// PRUNING
	// if multiple points are close together and similar velocity then combine them
	for (auto it = begin(mControlPoints); it != end(mControlPoints); ++it)
	{
		for (auto jt = it + 1; jt != end(mControlPoints);)
		{
			if (it->pos.distanceSquared(jt->pos) < mDistancePruneThreshold && it->vel.distanceSquared(jt->vel) < mControlPointVelocityPruneThreshold)
			{
				it->framesSeen = max(it->framesSeen, jt->framesSeen);
				it->framesNotSeen = min(it->framesNotSeen, jt->framesNotSeen);
				it->birthTime = min(it->birthTime, jt->birthTime);
				it->isActive = it->isActive || jt->isActive;
				jt = mControlPoints.erase(jt);
			}
			else
			{
				++jt;
			}
		}
	}
	// remove newest one until we don't have too many
	sort(begin(mControlPoints), end(mControlPoints), [](ControlPoint const& lhs, ControlPoint const& rhs) {
		return lhs.framesSeen > rhs.framesSeen;
	});
	mControlPoints = vector<ControlPoint>(begin(mControlPoints), begin(mControlPoints) + (min(mControlPoints.size(), (size_t)mMaxNumControlPoints)));
}


void ControlPointTracker::transformControlPoints()
{
	mUntransformedControlPoints.clear();
	mUntransformedControlPoints.reserve(mControlPoints.size());
	//float scaleFromUserCentroid3d = gInput->mapKinectToWorld(scaleFromUserCentroid);
	float amountToIncreaseY = (1.f - scaleFromUserCentroid) * mIncreaseHeightFromDistanceAndScale;
	Vec3f centroidWorld = gInput->userStats.getUserStats().centroid;
	Vec2f centroid2d = gInput->mapWorldToKinect(centroidWorld);
	float userDistance = centroidWorld.z;
	//float meanDistance = [&] {
	//	float s(0.f);
	//	for (ControlPoint const& c : mControlPoints)
	//		s += c.centroid3d.z;
	//	s /= mControlPoints.size();
	//	return s;
	//}();
	for (ControlPoint & c : mControlPoints)
	{
		mUntransformedControlPoints.emplace_back(c.pos2d);
		c.pos2d = centroid2d + (c.pos2d - centroid2d) * scaleFromUserCentroid;
		/// y points down in 2d points as they are in kinect coordinates
		c.pos2d.y -= amountToIncreaseY * (userDistance - mDistanceCenter);
		c.pos = centroidWorld + (c.pos - centroidWorld) * scaleFromUserCentroid;
		/// y points up in world points
		c.pos.y += amountToIncreaseY * (userDistance - mDistanceCenter);
	}
}


void ControlPointTracker::update(cv::Mat const& depthImage, cv::Mat const& bodyMask, cv::Mat const& skeletonImage, vector<Contour> const& contours)
{
	assert(skeletonImage.type() == CV_8UC1);
	assert(depthImage.type() == CV_16UC1);
	static int const t = bodyMask.type();
	static int const t1 = CV_8UC1;
	static int const t2 = CV_16UC1;
	static int const t3 = CV_32F;
	assert(bodyMask.type() == CV_8UC1);
	assert(skeletonImage.size() == depthImage.size());
	assert(skeletonImage.size() == bodyMask.size());


	findRawPoints(skeletonImage, contours);
	clusterRawPoints();
	estimateControlPointPositions(depthImage, bodyMask);
	updateControlPoints();
	pruneControlPoints();
	transformControlPoints();

	{
		UniqueLock lock(mMutex);
		pImageSize = fromOcv(skeletonImage.size());
		pRawControlPoints = mRawPoints;
		pRawControlPointClusters = mClusteredRawPoints;
		pPrevControlPoints2d = mPrevControlPoints2d;
		pEstimatedControlPoints2d = mEstimatedPrevControlPoints2d;
		pUntransformedControlPoints = mUntransformedControlPoints;
		pControlPoints = mControlPoints;
	}
}


void ControlPointTracker::draw()
{
	gl::enableAlphaBlending();

	if (mDrawRawControlPoints)
	{
		SharedLock lock(mMutex);
		gInput->setMatricesKinect(pImageSize, true);
		for (int i = 0; i < pRawControlPoints.size(); i++)
		{
			gl::color(getDebugColor(i, 0.75f));
			gl::drawSolidEllipse(gInput->mapWorldToKinect(pRawControlPoints[i]), 4, 7, 10);
		}
	}

	if (mDrawEstimatedControlPoints)
	{
		SharedLock lock(mMutex);
		gInput->setMatricesKinect(pImageSize, true);
		for (int i = 0; i < pEstimatedControlPoints2d.size(); i++)
		{
			int id = pEstimatedControlPoints2d[i].first;
			// if necessary, check to find we have a corresponding active control point
			if (!mDrawInactiveControlPoints)
			{
				auto it = find_if(begin(pControlPoints), end(pControlPoints), [&](ControlPoint const& c) {
					return c.id == id;
				});
				// skip if no corresponding control point
				if (it == end(pControlPoints))
				{
					continue;
				}
			}
			gl::color(getDebugColor(id, 1.f));
			Vec2f const& p = pEstimatedControlPoints2d[i].second;
			gl::drawLine(p - Vec2f(5, 5), p + Vec2f(5, 5));
			gl::drawLine(p - Vec2f(5, -5), p + Vec2f(5, -5));
		}
	}

	if (mDrawRawControlPointClusters)
	{
		SharedLock lock(mMutex);
		gInput->setMatricesKinect(pImageSize, true);
		for (int i = 0; i < pRawControlPointClusters.size(); i++)
		{
			gl::color(getDebugColor(i, 0.8f));
			gl::drawSolidEllipse(gInput->mapWorldToKinect(pRawControlPointClusters[i]), 7, 4, 10);
		}
	}

	if (mDrawControlPoints3d || mDrawControlPoints2d || mDrawUntransformedControlPoints)
	{
		float textLineHeight = gDebugFontMedium->measureString("x").y;
		Vec2f textPos = gRenderArea.getUpperLeft() + Vec2f(textLineHeight / 2, textLineHeight * 2);
		gl::setMatricesWindow(gRenderTargetSize, true);
		gDebugFontMedium->drawString("CONTROL POINTS", textPos);
		textPos.y += textLineHeight;
		SharedLock lock(mMutex);
		for (int i = 0; i < pControlPoints.size(); i++)
		{
			bool isActive = pControlPoints[i].isActive;
			if (!isActive && !mDrawInactiveControlPoints)
			{
				continue;
			}
			gl::color(getDebugColor(pControlPoints[i].id, isActive ? 1.f : 0.2f));
			gInput->setMatricesKinect(pImageSize, true);
			float radius = 4;
			if (mDrawDynamicSpeed)
			{
				radius += 6 * pControlPoints[i].dynamicSpeed.value;
			}
			if (mDrawDynamicAccel)
			{
				radius += 6 * pControlPoints[i].dynamicAccel.value;
			}
			if (mDrawControlPoints3d)
			{
				Vec2f pos = gInput->mapWorldToKinect(pControlPoints[i].pos);
				gl::drawSolidCircle(pos, radius, 10);
				gl::drawStrokedCircle(pos, 4, 10);
			}
			if (mDrawControlPoints2d)
			{
				Vec2f pos = pControlPoints[i].pos2d;
				gl::drawSolidCircle(pos, radius, 10);
				gl::drawStrokedCircle(pos, 4, 10);
				//app::console() << pControlPoints[i].pos2d << endl;
			}
			if (mDrawUntransformedControlPoints)
			{
				gl::drawStrokedCircle(pUntransformedControlPoints[i], 4, 10);
				if (mDrawControlPoints2d)
				{
					gl::drawLine(pUntransformedControlPoints[i], pControlPoints[i].pos2d);
				}
			}
			gl::setMatricesWindow(gRenderTargetSize);
			stringstream extraText;
			extraText << " " << setprecision(5) << pControlPoints[i].dynamicSpeed.value << "   " << pControlPoints[i].dynamicAccel.value << ' ';
			extraText << '(' << right << setw(5) << setprecision(4) << pControlPoints[i].pos2d.x << ','<< setw(5) << pControlPoints[i].pos2d.y << ')';
			gDebugFontMedium->drawString(to_string(pControlPoints[i].id) + "  " + to_string(pControlPoints[i].framesSeen)+extraText.str(), textPos);
			textPos.y += textLineHeight;

			if (mDrawDynamicSpeed)
			{
				pControlPoints[i].dynamicSpeedGrapher.draw();
			}
			if (mDrawDynamicAccel)
			{
				pControlPoints[i].dynamicAccelGrapher.draw();
			}
			if (mDrawKalmanSpeed)
			{
				pControlPoints[i].kalmanSpeedGrapher.draw();
			}
		}
	}

	if (mDrawEstimatedControlPoints)
	{
		SharedLock lock(mMutex);
		gInput->setMatricesKinect(pImageSize, true);
		assert(pPrevControlPoints2d.size() == pEstimatedControlPoints2d.size());
		for (int i = 0; i < pPrevControlPoints2d.size(); i++)
		{
			assert(pPrevControlPoints2d[i].first == pEstimatedControlPoints2d[i].first);
			auto col = getDebugColor(pPrevControlPoints2d[i].first);
			auto const& s = pPrevControlPoints2d[i].second;
			auto const& d = pEstimatedControlPoints2d[i].second;
			gl::color(col);
			gl::drawLine(s, d);
		}
	}
}