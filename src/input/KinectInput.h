#pragma once

#include "Kinect2.h"
#include "CinderOpenCV.h"
#include <boost/thread.hpp>

class KinectInput
{
public:
	KinectInput();
	virtual ~KinectInput();

	void start();
	void stop();

	//Kinect2::BodyIndexFrame getBodyIndex() const;
	/// returns shared data
	ci::Channel8u getBodyIndex() const;
	double getBodyIndexTimestamp() const;
	/// returns shared data
	ci::Surface8u getColorizedBodyIndex() const;
	Kinect2::BodyFrame getBody() const;
	//Kinect2::DepthFrame getDepth() const;
	/// Returns shared data
	ci::Channel16u getDepthChannel() const;
	double getDepthTimestamp() const;
	/// CV_32FC3. Returns clone.
	cv::Mat getPointCloud() const;
	Kinect2::DeviceRef getKinect() const;
	ci::Vec2i getImageSize() const;

private:
	void depthCallback(Kinect2::DepthFrame const& frame);
	void bodyCallback(Kinect2::BodyFrame const& frame);
	void bodyIndexCallback(Kinect2::BodyIndexFrame const& frame);
	/// Returns CV_32FC2 matrix of the given size.
	static cv::Mat createDepthToWorldSpaceMap(Kinect2::DeviceRef kinect, ci::Vec2i const& size);

	Kinect2::DeviceRef mKinect;
	std::unique_ptr<boost::thread> mThread;
	cv::Mat mDepthToWorldMappingTable;
	bool mIsRunning;

	mutable boost::shared_mutex mMutex;
	// guards the following
	ci::Vec2i mKinectImageSize;
	//Kinect2::BodyIndexFrame mBodyIndex;
	ci::Surface mColorizedBodyIndex;
	Kinect2::BodyFrame mBody;
	//Kinect2::DepthFrame mDepth;
	ci::Channel16u mDepth;
	double mDepthTimestamp;
	ci::Channel8u mBodyIndex;
	double mBodyIndexTimestamp;
	cv::Mat mPointCloud; ///< CV_32FC3, In world coordinates

};

