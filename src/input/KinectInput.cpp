#include "KinectInput.h"
#include "Common.h"
#include "Globals.h"
#include "utils/Profiler.h"

using namespace std;
using namespace ci;

typedef boost::shared_lock<boost::shared_mutex> SharedLock;
typedef boost::unique_lock<boost::shared_mutex> UniqueLock;

KinectInput::KinectInput()
: mIsRunning(false)
, mKinectImageSize(512, 424)
, mDepth(512, 424)
, mBodyIndex(512, 424)
, mColorizedBodyIndex(512, 424, false)
, mPointCloud(424, 512, CV_32FC3, cv::Scalar(0, 0, 0))
{

}

KinectInput::~KinectInput()
{
	stop();
}

void KinectInput::start()
{
	mIsRunning = true;
	mThread = unique_ptr<boost::thread>(new boost::thread([this]() {
		while (mKinect == nullptr && mIsRunning)
		{
			try
			{
				{
					UniqueLock lock(mMutex);
					mKinect = Kinect2::Device::create();
				}
				mKinect->connectBodyEventHandler(&KinectInput::bodyCallback, this);
				mKinect->connectBodyIndexEventHandler(&KinectInput::bodyIndexCallback, this);
				mKinect->connectDepthEventHandler(&KinectInput::depthCallback, this);
				mKinect->start();
			}
			catch (Kinect2::Device::Exception const& e)
			{
				cerr << g.time << " Error opening Kinect: " << e.what() << endl;
				mKinect = nullptr;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
			}
		}
		while (mIsRunning)
		{
			mKinect->update();
		}
		mKinect->disconnectBodyEventHandler();
		mKinect->disconnectBodyIndexEventHandler();
		mKinect->disconnectDepthEventHandler();

		mKinect = nullptr;

		cout << "Kinect input thread completing." << endl;
	}));
}

void KinectInput::stop()
{
	mIsRunning = false;
	if (mThread != nullptr)
	{
		mThread->join();
		mThread = nullptr;
	}
}


ci::Channel8u KinectInput::getBodyIndex() const
{
	SharedLock lock(mMutex);
	return mBodyIndex;
}

ci::Surface8u KinectInput::getColorizedBodyIndex() const
{
	SharedLock lock(mMutex);
	return mColorizedBodyIndex;
}

double KinectInput::getBodyIndexTimestamp() const
{
	SharedLock lock(mMutex);
	return mBodyIndexTimestamp;
}

Kinect2::BodyFrame KinectInput::getBody() const
{
	SharedLock lock(mMutex);
	return mBody;
}

//Kinect2::DepthFrame KinectInput::getDepth() const
//{
//	SharedLock lock(mMutex);
//	return mDepth;
//}

ci::Channel16u KinectInput::getDepthChannel() const
{
	SharedLock lock(mMutex);
	//return mDepth.getChannel()? mDepth.getChannel().clone() : ci::Channel16u();
	return mDepth;
}

double KinectInput::getDepthTimestamp() const
{
	SharedLock lock(mMutex);
	//return mDepth.getTimeStamp();
	return mDepthTimestamp;
}

/// CV_32FC3
cv::Mat KinectInput::getPointCloud() const
{
	START(KinectInput::getPointCloud_acquire);
	SharedLock lock(mMutex);
	END(KinectInput::getPointCloud_acquire);
	return mPointCloud.clone();
}

Kinect2::DeviceRef KinectInput::getKinect() const
{
	SharedLock lock(mMutex);
	return mKinect;
}

ci::Vec2i KinectInput::getImageSize() const
{
	SharedLock lock(mMutex);
	return mKinectImageSize;
}


void KinectInput::bodyCallback(Kinect2::BodyFrame const& frame)
{
	// TODO: Reflect x axis
	UniqueLock lock(mMutex);
	mBody = frame;
}

void KinectInput::bodyIndexCallback(Kinect2::BodyIndexFrame const& frame)
{
	cv::Mat bodyIndex = toOcv(frame.getChannel());
	cv::flip(bodyIndex, bodyIndex, 1);
	ci::Surface colorizedBodyIndex = Kinect2::colorizeBodyIndex(mBodyIndex);
	UniqueLock lock(mMutex);
	if (!gInput->isInputPaused)
	{
		mKinectImageSize = frame.getChannel().getSize();
		//mBodyIndex = frame.getChannel().clone();
		//cv::flip(toOcv(mBodyIndex), toOcv(mBodyIndex), 1);
		//mBodyIndex = ci::Channel(frame.getChannel().getWidth(), frame.getChannel().getHeight());
		mBodyIndex = fromOcv(bodyIndex);
	}

	mBodyIndexTimestamp = frame.getTimeStamp();
	mColorizedBodyIndex = colorizedBodyIndex;
}

void KinectInput::depthCallback(Kinect2::DepthFrame const& frame)
{
	if (mDepthToWorldMappingTable.rows == 0 || mDepthToWorldMappingTable.cols == 0 || fromOcv(mDepthToWorldMappingTable.size()) != frame.getSize())
	{
		mDepthToWorldMappingTable = createDepthToWorldSpaceMap(mKinect, frame.getSize());
	}
	{
		// get as much work done as possible before the lock
		cv::Mat depth = toOcv(frame.getChannel());
		cv::flip(depth, depth, 1);

		cv::Mat pointCloud;
		if (depth.size() == mDepthToWorldMappingTable.size())
		{
			pointCloud.create(depth.size(), CV_32FC3);
			//int i = 0;
			//cv::Mat depthMat = toOcv(mDepth);
			auto it_cloud(pointCloud.begin<cv::Vec3f>()), end_cloud(pointCloud.end<cv::Vec3f>());
			auto it_depth(depth.begin<uint16_t>()), end_depth(depth.end<uint16_t>());
			auto it_map(mDepthToWorldMappingTable.begin<Vec2f>()), end_map(mDepthToWorldMappingTable.end<Vec2f>());
			for (; it_cloud != end_cloud; ++it_cloud, ++it_depth, ++it_map)
			{
				assert(it_depth != end_depth);
				assert(it_map != end_map);
				// convert all values from mm to m
				*it_cloud = cv::Vec3f(*it_depth * (*it_map).x * 0.001f, *it_depth * (*it_map).y * 0.001f, *it_depth * 0.001f);
			}
		}


		UniqueLock lock(mMutex);
		if (!gInput->isInputPaused)
		{
			mKinectImageSize = frame.getChannel().getSize();
			mDepth = ci::Channel16u(depth.cols, depth.rows);
			mDepth = fromOcv(depth);
		}
		mDepthTimestamp = frame.getTimeStamp();
		assert(mDepth.getSize() == fromOcv(mDepthToWorldMappingTable.size()));
		if (mPointCloud.rows && mPointCloud.cols)
		{
			//cv::swap(mPointCloud, pointCloud);
			mPointCloud = pointCloud;
		}
	}
}

cv::Mat KinectInput::createDepthToWorldSpaceMap(Kinect2::DeviceRef kinect, ci::Vec2i const& size)
{
	cv::Mat map(toOcv(size), CV_32FC2, cv::Scalar(0, 0));

	PointF* table = nullptr;
	uint32_t count = 0;
	long hr = GetDepthFrameToCameraSpaceTable(kinect->getHandle(), &count, &table);
	if (SUCCEEDED(hr)) {
		auto it_map(map.begin<cv::Vec2f>()), end_map(map.end<cv::Vec2f>());
		uint32_t i = 0;
		for (; it_map != end_map && i < count; ++it_map, ++i)
		{
			(*it_map)[0] = table[i].X;
			(*it_map)[1] = table[i].Y;
		}
	}
	return map;
}

