#include "input/Input.h"
#include "Common.h"
#include "cinder/gl/Texture.h"
#include "utils/MovingAverage.h"
#include <boost/format.hpp>
#include "input/SkeletonFinder.h"
#include <numeric>
#include "cinder/app/App.h"
#include "utils/Profiler.h"
#include "Globals.h"

using namespace std;
typedef boost::lock_guard<boost::mutex> Lock;
typedef boost::shared_lock<boost::shared_mutex> SharedLock;
typedef boost::unique_lock<boost::shared_mutex> UniqueLock;


// temp
double trackerTimeSum(0);
int trackerTimeCount(0);




Input::Input()
: mIsRunning(false)
, mKinectImageSize(640, 480)
, mContourTimestamp(-42)
//, mNextControlPointId(0)
{
#define PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "Input" }));
#define OPTION(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "Input" }, "unsaved"));

	OPTION(isInputPaused, false);

	PARAM(mDrawDepth, false);
	PARAM(mDrawMaskedDepth8u, false);
	PARAM(mDrawBodyIndex, false);
	PARAM(mDrawRawContours, false);
	PARAM(mDrawContours, false);
	PARAM(mDrawSmoothedContourAngles, false);
	PARAM(mDrawShapes, false);
	//PARAM(mDrawFilteredThinnedImage, false);
	PARAM(mDrawContourAveragedBodyIndex, false);

	// PARAMETERS
	PARAM(mKinectRenderArea.x1, -1);
	PARAM(mKinectRenderArea.y1, -1);
	PARAM(mKinectRenderArea.x2, +1);
	PARAM(mKinectRenderArea.y2, +1);
	PARAM(mProjectionWallDistance, 6);


	// contours
	PARAM(mContourBodyAveragePeriod, 2);
	PARAM(mMaxSkeletons, 4);
	PARAM(mEnableContourShapes, false);
	PARAM(mPreContourDilationAmount, 0);
	PARAM(mContourSmoothEpsilon, 1.0);
	PARAM(mMinContourArea, 1.0);
	PARAM(mMinCornerAngle, 0.5);
	PARAM(mMinTerminationAngle, 1.0);
	PARAM(mMaxShapes, 10);
	PARAM(mMinShapeExtent, 0.9);
	// skeleton finder
	PARAM(mSkeletonsPreciseProcess, true);
	PARAM(mConnectionThreshold, 0.f);
	// control point finder
	PARAM(mSkeletonSmoothingEpsilon, 1.f);

}


Input::~Input()
{
	stop();
}

void Input::start()
{
	kinect.start();
	flow.start();

	mIsRunning = true;


	mContourThread = unique_ptr<boost::thread>(new boost::thread([this](){ contourThreadFunction(); }));
}

/// Creates a subcontour shape. Args are indices. Points can wrap around the edges but only left XOR right
/// point can do this, not both. c is source contour.
Contour createShape(Contour const& c, int left, int center, int right, int numPoints)
{
	assert((left <= center ? center - left : center + c.points.size() - left)
		+ (right >= center ? right - center : right + c.points.size() - center)
		== numPoints);
	Contour shape;
	shape.points.reserve(numPoints);
	shape.angles.reserve(numPoints);
	// only one side can loop round
	assert(!(left > center && (right != 0 && right < center)));
	if (left > center)
	{
		copy(begin(c.points) + left, end(c.points), back_inserter<vector<cv::Point>>(shape.points));
		copy(begin(c.angles) + left, end(c.angles), back_inserter<vector<double>>(shape.angles));
		copy(begin(c.points), begin(c.points) + center, back_inserter<vector<cv::Point>>(shape.points));
		copy(begin(c.angles), begin(c.angles) + center, back_inserter<vector<double>>(shape.angles));
	}
	else
	{
		copy(begin(c.points) + left, begin(c.points) + center, back_inserter<vector<cv::Point>>(shape.points));
		copy(begin(c.angles) + left, begin(c.angles) + center, back_inserter<vector<double>>(shape.angles));
	}
	shape.points.push_back(c.points[center]);
	shape.angles.push_back(c.angles[center]);
	shape.apexIndex = shape.points.size() - 1;
	if (right < center)
	{
		copy(begin(c.points) + center, end(c.points), back_inserter<vector<cv::Point>>(shape.points));
		copy(begin(c.angles) + center, end(c.angles), back_inserter<vector<double>>(shape.angles));
		copy(begin(c.points), begin(c.points) + right, back_inserter<vector<cv::Point>>(shape.points));
		copy(begin(c.angles), begin(c.angles) + right, back_inserter<vector<double>>(shape.angles));
	}
	else
	{
		copy(begin(c.points) + center, begin(c.points) + right, back_inserter<vector<cv::Point>>(shape.points));
		copy(begin(c.angles) + center, begin(c.angles) + right, back_inserter<vector<double>>(shape.angles));
	}
	return shape;
}

void Input::contourThreadFunction()
{
	while (mIsRunning)
	{
		//Kinect2::BodyIndexFrame bodyIndexFrame = kinect.getBodyIndex();
		//ci::Surface
		//double timestamp = bodyIndexFrame.getTimeStamp();
		double timestamp = min(kinect.getDepthTimestamp(), kinect.getBodyIndexTimestamp());

		if (mContourTimestamp < timestamp)
		{
			cv::Mat bodyIndex;
			cv::Mat depthImage;
			cv::Mat pointCloud = kinect.getPointCloud();
			{
				//Channel channel = bodyIndexFrame.getChannel();
				Channel channel = kinect.getBodyIndex();
				if (channel)
				{
					bodyIndex = toOcv(channel).clone();
				}
			}
			{
				Channel16u channel = kinect.getDepthChannel();
				if (channel)
				{
					depthImage = toOcv(channel).clone();
				}
			}
			if (bodyIndex.rows > 0 && bodyIndex.cols > 0
				&& depthImage.rows > 0 && depthImage.cols > 0
				&& pointCloud.rows > 0 && pointCloud.cols > 0
				&& bodyIndex.size() == depthImage.size()
				&& bodyIndex.size() == pointCloud.size())
			{
				// send pixels above 250 to zero. these are the background.
				cv::threshold(bodyIndex, bodyIndex, 250, 255, CV_THRESH_BINARY_INV);
				// average over previous values
				{
					mContourBodyAveragePeriod = clamp(mContourBodyAveragePeriod, 0, 150);
					mPrevBodyIndexes.set_capacity(mContourBodyAveragePeriod);
					if (!mPrevBodyIndexes.empty() && (mPrevBodyIndexes.front().type() != bodyIndex.type() || mPrevBodyIndexes.front().size() != bodyIndex.size()))
					{
						mPrevBodyIndexes.clear();
					}
					cv::Mat copy = bodyIndex.clone();
					// average over past instances
					for (cv::Mat const& mat : mPrevBodyIndexes)
					{
						//bodyIndex = max(bodyIndex, mat);
						cv::bitwise_or(bodyIndex, mat, bodyIndex);
					}
					mPrevBodyIndexes.push_back(copy);
					{
						Lock lock(mContourMutex);
						mContourAveragedBodyIndex = bodyIndex.clone();
					}
				}
				vector<Contour> contours = findContours(bodyIndex);

				cv::Mat maskedDepth8u(depthImage.size(), CV_8UC1);
				// process depth image by masking and reducing range
				{
					auto in(depthImage.begin<uint16_t>()), in_end(depthImage.end<uint16_t>());
					auto body(bodyIndex.begin<uint8_t>()), body_end(bodyIndex.end<uint8_t>());
					auto out(maskedDepth8u.begin<uint8_t>()), out_end(maskedDepth8u.end<uint8_t>());
					uint16_t farClip(1000 * mProjectionWallDistance);
					uint16_t nearClip(1000); // 1 metre
					uint16_t range = farClip - nearClip;
					/// divide by scale to go from range to 8 bit
					uint16_t scale = range / 255u - 1;
					for (; in != in_end; ++in, ++body, ++out)
					{
						// map input to 2m - projectionWallDistance
						*out = *body == 0 ? uint8_t(0) : /*cv::saturate_cast<uint8_t>*/((min(farClip, max(nearClip, *in)) - nearClip) / scale);
					}
				}

				//// Skeletons
				//SkeletonFinder skeletonFinder;
				//vector<cv::Rect> boundingBoxes;
				//vector<vector<cv::Point>> smoothedContourPoints;
				//int numSkeletons = min<int>(smoothedContours.size(), mMaxSkeletons);
				//smoothedContourPoints.reserve(numSkeletons);
				//boundingBoxes.reserve(numSkeletons);
				//for (int i = 0; i < numSkeletons; i++)
				//{
				//	smoothedContourPoints.push_back(smoothedContours[i].points);
				//	boundingBoxes.push_back(cv::boundingRect(smoothedContours[i].points));
				//}
				//cv::Mat skeletonImage = skeletonFinder.findSkeletons(bodyIndex.size(), smoothedContourPoints, boundingBoxes, mSkeletonsPreciseProcess, mConnectionThreshold);
				//vector<PolyLine2i> skeletons;
				//vector<ci::Vec2f> centroids;
				//skeletons.reserve(skeletonFinder.getSkeletons().size());
				//vector<PolyLine2i> rawSkeletons = skeletonFinder.getSkeletons();
				//vector<size_t> rawSkeletonIndices = skeletonFinder.getSkeletonContours();
				//assert(rawSkeletonIndices.size() == rawSkeletons.size());
				//for (int i = 0; i < rawSkeletons.size(); i++)
				//{
				//	PolyLine2i const& rawSkeleton = rawSkeletons[i];
				//	if (rawSkeleton.size() < 2)
				//	{
				//		continue;
				//	}
				//	vector<ci::Vec2i> const& raw = rawSkeleton.getPoints();
				//	vector<ci::Vec2i> smoothed;
				//	cv::approxPolyDP(reinterpret_cast<vector<cv::Point> const&>(raw), reinterpret_cast<vector<cv::Point>&>(smoothed), mSkeletonSmoothingEpsilon, false);
				//	skeletons.push_back(PolyLine2i(std::move(smoothed)));
				//	centroids.push_back(smoothedContours[rawSkeletonIndices[i]].centroid);
				//}

				skeletonFinder.update(bodyIndex, contours);
				cv::Mat skeletonImage = skeletonFinder.getSkeletonImage();
				tracker.update(depthImage, skeletonFinder.getContourMask(), skeletonImage, contours);

				// --- UPDATE USER STATS
				Contour const* largestContour = nullptr;
				{
					float largestArea = 0;
					for (Contour const& c : contours)
					{
						if (c.area > largestArea)
						{
							largestArea = c.area;
							largestContour = &c;
						}
					}
				}
				g.isTrackingUser = largestContour != nullptr;
				//UserStats userStats = largestContour == nullptr ? UserStats() : mUserStatsCalculator.update(pointCloud, bodyIndex, largestContour->boundingBox);
				//double t0 = app::getElapsedSeconds();
				userStats.update(pointCloud, bodyIndex, maskedDepth8u, largestContour == nullptr ? cv::Rect() : largestContour->boundingBox);
				//double t1 = app::getElapsedSeconds();
				//double t = t1 - t0;
				//trackerTimeSum += t;
				//trackerTimeCount++;
				//cout << "userstats: " << t << " mean: " << (trackerTimeSum / trackerTimeCount) << endl;
				{
					Lock lock(mContourMutex);
					mSmoothedContours = contours;
					mContourImageSize = fromOcv(bodyIndex.size());
					////mSkeletons = skeletons;
					//mRawControlPointClusters = mRawControlPointClustersInternal;
					//mRawControlPoints = mRawControlPointsInternal;
					////mEstimatedControlPoints = mEstimatedControlPointsInternal;
					//mControlPoints = mControlPointsInternal;
					mThinnedImage = skeletonImage;
					mMaskedDepth8u = maskedDepth8u;
					//mUserBoundingBoxes = userBoundingBoxes;
					//mFilteredThinnedImage = filteredSkeletonImage;
					//mUserStats = userStats;
				}
			}
			mContourTimestamp = timestamp;
		}
	}
}


vector<Contour> Input::findContours(cv::Mat const& bodyIndex) const
{
	cv::Mat dilated;
	cv::dilate(bodyIndex, dilated, cv::Mat(), cv::Point(-1, -1), clamp(mPreContourDilationAmount, 0, 100));
	vector<vector<cv::Point>> contours;
	cv::findContours(dilated, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	vector<Contour> smoothedContours;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() <= 2)
		{
			continue;
		}
		Contour c;
		c.rawPoints = contours[i];
		cv::approxPolyDP(c.rawPoints, c.points, mContourSmoothEpsilon, true);
		// signed area. negative area means the contour is clockwise.
		// we want all contours to be anticlockwise
		//c.area = cv::contourArea(c.points, true);
		c.boundingBox = cv::boundingRect(c.points);
		cv::Moments m = cv::moments(c.points);
		c.centroid = Vec2f(m.m10 / m.m00, m.m01 / m.m00);
		c.area = m.m00;
		if (abs(c.area) >= mMinContourArea)
		{
			if (c.area < 0)
			{
				std::reverse(c.points.begin(), c.points.end());
				c.area = -c.area;
			}
			if (mEnableContourShapes)
			{
				findContourShapes(c);
			}
			smoothedContours.push_back(c);
		}
	}
	return smoothedContours;
}



void Input::findContourShapes(Contour & c) const
{
	c.angles.reserve(c.points.size());
	cv::Point prevLine = c.points.front() - c.points.back();
	// angle to x axis, -pi to +pi
	double prevAngle = atan2(prevLine.y, prevLine.x);
	static float const PI = float(M_PI);
	for (int i = 0; i < c.points.size() - 1; i++)
	{
		cv::Point line = c.points[i + 1] - c.points[i];
		// angle to x axis
		double angle = atan2(line.y, line.x);
		double cornerAngle = angle - prevAngle;
		// put angles in range (-pi, pi]
		cornerAngle = cornerAngle <= -PI ? cornerAngle + 2 * PI : cornerAngle > PI ? cornerAngle - 2 * PI : cornerAngle;
		c.angles.push_back(cornerAngle);
		prevLine = line;
		prevAngle = angle;
	}
	// last point loops back to start
	cv::Point line = c.points.front() - c.points.back();
	double angle = atan2(line.y, line.x);
	double cornerAngle = angle - prevAngle;
	cornerAngle = cornerAngle <= -PI ? cornerAngle + 2 * PI : cornerAngle > PI ? cornerAngle - 2 * PI : cornerAngle;
	c.angles.push_back(cornerAngle);
	assert(c.points.size() == c.angles.size());
	// now find candidate shapes by starting at the 10 biggest angles and growing a shape on each side until we reach an angle
	// that is less than -minTerminationAngle (this will be winding the wrong direction and greater than minTerminationAngle).
	// if we get back to the start then we reject the candidate shape and stop the search
	vector<size_t> indices;
	indices.reserve(c.angles.size());
	for (int i = 0; i < c.angles.size(); i++)
	{
		indices.push_back(i);
	}
	// order indices so largest angle is at front
	std::sort(indices.begin(), indices.end(), [&](size_t lhs, size_t rhs) {
		return c.angles[lhs] > c.angles[rhs];
	});
	int n = indices.size();
	for (int k = 0; k < min<int>(indices.size(), mMaxShapes); k++)
	{
		int index = indices[k];
		if (c.angles[index] < mMinCornerAngle)
		{
			break;
		}
		int left = mod(index - 1, n);
		int right = mod(index + 1, n);
		int count = 2;
		bool moveLeft(true), moveRight(true);
		while (moveLeft && moveRight)
		{
			if (count > 3)
			{
				Contour contour = createShape(c, left, index, right, count);
				if (cv::contourArea(contour.points) / cv::boundingRect(contour.points).area() < mMinShapeExtent)
				{
					break;
				}
			}

			if (left == right)
			{
				break;
			}
			if (moveLeft)
			{
				left = mod(left - 1, n);
				count++;
			}
			moveLeft = c.angles[left] > -mMinTerminationAngle;
			if (left == right)
			{
				break;
			}
			if (moveRight)
			{
				right = mod(right + 1, n);
				count++;
			}
			moveRight = c.angles[right] > -mMinTerminationAngle;
		}

		if (count < 4)
		{
			continue;
		}
		Contour shape = createShape(c, left, index, right, count);
		c.shapes.push_back(shape);
	}
}



void Input::stop()
{
	mIsRunning = false;
	flow.stop();
	if (mContourThread != nullptr)
	{
		mContourThread->join();
		mContourThread = nullptr;
	}
	kinect.stop();
}




void Input::update()
{
	PROFILE(Input::update);
	//START(Input::update_getKinectImageSize);
	//mKinectImageSize = kinect.getImageSize();
	//END(Input::update_getKinectImageSize);
	{
		//START(Input::update_lock);
		UniqueLock lock(mLastPointCloudMutex);
		//END(Input::update_lock);
		mLastPointCloud = kinect.getPointCloud();
		mKinectImageSize.set(max(1, mLastPointCloud.cols), max(1, mLastPointCloud.rows));
		assert(mKinectImageSize == kinect.getImageSize());
	}
	mKinectImageSize.x = max(mKinectImageSize.x, 1);
	mKinectImageSize.y = max(mKinectImageSize.y, 1);
	mKinectToRenderMapping = RectMapping(Rectf(0, mKinectImageSize.y, mKinectImageSize.x, 0), mKinectRenderArea);
	mRenderToKinectMapping = RectMapping(mKinectRenderArea, Rectf(0, mKinectImageSize.y, mKinectImageSize.x, 0));
	mKinectAligner.update();
}


void Input::setMatricesKinect(Vec2i const& size, bool originUpperLeft) const
{
	setMatricesRender();
	gl::translate(mKinectRenderArea.getUpperLeft());
	// origin is now at the lower left of kinect render area (the lower y coordinate)
	gl::scale(Vec2f(1, 1) / Vec2f(size) * Vec2f(mKinectRenderArea.getSize()));
	if (originUpperLeft)
	{
		gl::translate(0, size.y, 0);
		gl::scale(1, -1, 1);
	}
	// drawing from origin up to size now renders into the kinect render area

}

void Input::draw()
{
	// matrices will be set to render or 3d view when this function is called.
	//setMatricesRender();
	gl::enableAlphaBlending();
	Rectf invertedKinectArea = getKinectArea();
	swap(invertedKinectArea.y1, invertedKinectArea.y2);
	if (mDrawDepth)
	{
		Channel16u const& channel16u = kinect.getDepthChannel();
		if (channel16u)
		{
			Channel8u channel = Kinect2::channel16To8(channel16u);
			// need to render upside down as images have y going downwards
			gl::draw(gl::Texture(channel), /*Area(0, channel.getHeight(), channel.getWidth(), 0),*/ invertedKinectArea);
		}
	}

	if (mDrawMaskedDepth8u)
	{
		Lock lock(mContourMutex);
		if (mMaskedDepth8u.cols && mMaskedDepth8u.rows)
		{
			gl::draw(gl::Texture(fromOcv(mMaskedDepth8u)), invertedKinectArea);
		}
	}

	if (mDrawBodyIndex)
	{
		gl::color(ColorA(1, 1, 1, 0.75));
		Surface colorizedBodyIndex = kinect.getColorizedBodyIndex();
		if (colorizedBodyIndex)
		{
			gl::draw(gl::Texture(colorizedBodyIndex), invertedKinectArea);
		}
	}

	if (mDrawContourAveragedBodyIndex)
	{
		Lock lock(mContourMutex);
		gl::color(ColorA(1, 1, 0, 0.75));
		gl::draw(gl::Texture(fromOcv(mContourAveragedBodyIndex)), invertedKinectArea);
	}

	userStats.draw();
	flow.draw();

	gl::enableAlphaBlending();
	glDisable(GL_TEXTURE_2D);

	if (mDrawRawContours)
	{
		Lock lock(mContourMutex);
		setMatricesKinect(mContourImageSize, true);
		for (int i = 0; i < mSmoothedContours.size(); i++)
		{
			vector<cv::Point> const& contour = mSmoothedContours[i].rawPoints;
			gl::color(gDebugColors[i%gDebugColors.size()]);
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(2, GL_INT, 0, contour.data());
			glDrawArrays(GL_LINE_STRIP, 0, contour.size());
			glDisableClientState(GL_VERTEX_ARRAY);
		}
		//cout << "num contours: " << mContours.size() << endl;
	}

	if (mDrawContours)
	{
		Lock lock(mContourMutex);
		setMatricesKinect(mContourImageSize, true);
		for (int i = 0; i < mSmoothedContours.size(); i++)
		{
			Contour const& contour = mSmoothedContours[i];
			gl::color(gDebugColors[i%gDebugColors.size()]);
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(2, GL_INT, 0, contour.points.data());
			glDrawArrays(GL_LINE_STRIP, 0, contour.points.size());
			glDisableClientState(GL_VERTEX_ARRAY);
			// draw angles
			if (mDrawSmoothedContourAngles)
			{
				for (int j = 0; j < contour.points.size(); j++)
				{
					double a = contour.angles[j];
					if (abs(a) > mMinCornerAngle)
					{
						gDebugFont->drawString(boost::str(boost::format("%.2f") % a), Vec2f(fromOcv(contour.points[j])));
					}
				}
			}
		}
	}
	if (mDrawShapes)
	{
		Lock lock(mContourMutex);
		setMatricesKinect(mContourImageSize, true);
		{
			for (int i = 0; i < mSmoothedContours.size(); i++)
			{
				Contour const& contour = mSmoothedContours[i];
				for (int j = 0; j < contour.shapes.size(); j++)
				{
					Contour const& shape = contour.shapes[j];
					gl::color(gDebugColors[(i + j) % gDebugColors.size()]);
					vector<cv::Point> points = shape.points;
					points.push_back(points.front());
					glEnableClientState(GL_VERTEX_ARRAY);
					glVertexPointer(2, GL_INT, 0, points.data());
					glDrawArrays(GL_LINE_STRIP, 0, points.size());
					glDisableClientState(GL_VERTEX_ARRAY);
					if (shape.apexIndex >= 0)
					{
						gl::drawSolidCircle(Vec2f(fromOcv(shape.points.at(shape.apexIndex))), 3.0, 10);
					}
				}
			}
		}
	}

	tracker.draw();
	skeletonFinder.draw();
	mKinectAligner.draw();
}


// recursive helper function with no mutex
Vec3f p_mapKinectToWorld(Vec2i const& v, bool filterNoise, cv::Mat const& pointCloud)
{
	// this check should have been done by Input::mapKinectToWorld(Vec2i...)
	assert(pointCloud.rows > 0 && pointCloud.cols > 0 && pointCloud.type() == CV_32FC3);

	Vec3f p = fromOcv(pointCloud.at<cv::Vec3f>(cv::Point(
		cv::borderInterpolate(v.x, pointCloud.cols, cv::BORDER_REPLICATE),
		cv::borderInterpolate(v.y, pointCloud.rows, cv::BORDER_REPLICATE))));
	if (!filterNoise)
	{
		return p;
	}
	else
	{
		if (p.z == 0.f)
		{
			static Vec2i const neighborOffsets[8] = {
				Vec2i(-1, -1),
				Vec2i(+0, -1),
				Vec2i(+1, -1),
				Vec2i(+1, +0),
				Vec2i(+1, +1),
				Vec2i(+0, +1),
				Vec2i(-1, +1),
				Vec2i(-1, +0),
			};
			// try to find a value that doesn't have depth value as zero
			for (int i = 0; i < 8; i++)
			{
				Vec3f p2 = p_mapKinectToWorld(v + neighborOffsets[i], false, pointCloud);
				if (p2.z != 0.f)
				{
					p.z = p2.z;
					break;
				}
			}
		}
		// failed to find point or p.z != 0.f
		return p;
	}
}



Vec3f Input::mapKinectToWorld(Vec2i const& v, bool filterNoise) const
{
	SharedLock lock(mLastPointCloudMutex);
	if (mLastPointCloud.rows == 0 || mLastPointCloud.type() != CV_32FC3)
	{
		return Vec3f(v.x, v.y, 0);
	}
	return p_mapKinectToWorld(v, filterNoise, mLastPointCloud);
}


Vec3f Input::mapKinectToWorld(Vec2f const& v) const
{
	Vec2i lowerBound((int)v.x, (int)v.y);
	Vec2i upperBound = lowerBound + Vec2i(1, 1);

	//bilinear interpolation
	float dx = v.x - lowerBound.x;
	float dy = v.y - lowerBound.y;

	float weight_tl = (1.0f - dx) * (1.0f - dy);
	float weight_tr = (dx)* (1.0f - dy);
	float weight_bl = (1.0f - dx) * (dy);
	float weight_br = (dx)* (dy);

	return weight_tl * mapKinectToWorld(Vec2i(lowerBound.x, lowerBound.y))
		+ weight_tr * mapKinectToWorld(Vec2i(upperBound.x, lowerBound.y))
		+ weight_bl * mapKinectToWorld(Vec2i(lowerBound.x, upperBound.y))
		+ weight_br * mapKinectToWorld(Vec2i(upperBound.x, upperBound.y));
}


Vec2i Input::mapWorldToKinect(Vec3f const& v) const
{
	return kinect.getKinect()->mapCameraToDepth(v);
}

vector<Vec2i> Input::mapWorldToKinect(vector<Vec3f> const& v) const
{
	return kinect.getKinect()->mapCameraToDepth(v);
}

Vec3f Input::mapWorldToRender(Vec3f const& v) const
{
	Vec2i k = mapWorldToKinect(v);
	return mapKinectToRender(Vec3f(k.x, k.y, v.z));
}

Vec2f Input::mapWorldToRender(Vec2f const& v) const
{
	return mapWorldToRender(Vec3f(v, getProjectionWallDistance())).xy();
}