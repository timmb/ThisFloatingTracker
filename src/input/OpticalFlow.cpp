#include "OpticalFlow.h"
#include "Common.h"

using namespace ci;
using namespace std;

OpticalFlow::OpticalFlow()
: mIsRunning(false)
, mFlowTimestamp(-42)
{
#define PARAM(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "Flow" }));
#define OPTION(var, init) 	var = init; pars.addParam(new Parameter<decltype(var)>(&var, #var, { "Flow" }, "unsaved"));

	PARAM(mDrawRawFlow, false);
	PARAM(mDrawFlow, false);
	PARAM(mFlowDrawScale, 1.f);

	// flow
	PARAM(mFlowPreResize.x, 120);
	PARAM(mFlowPreResize.y, 90);
	PARAM(mFlowPreBlur, 2);
	PARAM(mFlowMinLength, 1.4);
	PARAM(mFlowMaxLength, 10);
	//PARAM(mFlowFilter, 20);
	PARAM(mFlowMovingAverage.mNumSamples, 40);


}


OpticalFlow::~OpticalFlow()
{
	stop();
}


void OpticalFlow::start()
{
	mIsRunning = true;
	mThread = unique_ptr<boost::thread>(new boost::thread([this]() {
		threadFunction();
	}));
}

void OpticalFlow::stop()
{
	mIsRunning = false;
	if (mThread != nullptr)
	{
		mThread->join();
		mThread = nullptr;
	}
}


void OpticalFlow::threadFunction()
{
	assert(gInput != nullptr);
	while (mIsRunning)
	{
		//Kinect2::BodyIndexFrame bodyIndexFrame = gInput->kinect.getBodyIndex();

		//double timestamp = bodyIndexFrame.getTimeStamp();

		double timestamp = gInput->kinect.getBodyIndexTimestamp();
		cv::Mat bodyIndex;
		if (timestamp > mFlowTimestamp)
		{
			//Channel channel = bodyIndexFrame.getChannel();
			Channel channel = gInput->kinect.getBodyIndex();
			if (channel)
				cv::resize(toOcv(channel), bodyIndex, toOcv(mFlowPreResize), 0, 0, CV_INTER_NN);
		}
		if (timestamp > mFlowTimestamp && bodyIndex.rows && bodyIndex.cols)
		{
			cv::Mat blurred;
			cv::GaussianBlur(bodyIndex, blurred, cv::Size(mFlowPreBlur * 2 + 1, mFlowPreBlur * 2 + 1), 0);
			if (blurred.size() == mPrevResizedBodyIndex.size() && blurred.type() == mPrevResizedBodyIndex.type())
			{
				cv::Mat rawFlow;
				if (gInput->isInputPaused)
				{
					SharedLock lock(mMutex);
					rawFlow = mRawFlow.clone();
				}
				else
				{
					cv::calcOpticalFlowFarneback(mPrevResizedBodyIndex, blurred, rawFlow, 0.7, 6, 4, 5, 7, 2.2, false);
				}
				assert(rawFlow.rows && rawFlow.cols);
				// ok to read mFlow without locking the mutex as it is only modified by this thread
				cv::Mat flow(rawFlow.size(), CV_32FC2);
				if (rawFlow.rows && rawFlow.cols)
				{
					//// merge flow with prev flow using exponential filter and clamp
					//Lock lock(mFlowMutex);

					float minFlowLengthSq = mFlowMinLength * mFlowMinLength;
					float maxFlowLengthSq = mFlowMaxLength * mFlowMaxLength;
					//cv::Mat avgFlow;

					auto it_raw(rawFlow.begin<cv::Vec2f>()), end_raw(rawFlow.end<cv::Vec2f>());
					auto it_out(flow.begin<cv::Vec2f>()), end_out(flow.end<cv::Vec2f>());
					for (; it_raw != end_raw; ++it_raw, ++it_out)
					{
						assert(it_out != end_out);
						cv::Vec2f const& in = *it_raw;
						cv::Vec2f & out = *it_out;
						float lenSq = in[0] * in[0] + in[1] * in[1];
						if (lenSq < minFlowLengthSq || in[0] != in[0] || in[1] != in[1])
						{
							out = cv::Vec2f(0, 0);
						}
						else if (lenSq > maxFlowLengthSq)
						{
							out = in * (mFlowMaxLength * fastInvSqrt(lenSq));
						}
						else
						{
							out = in;
						}
					}
					if (!mFlowMovingAverage.isEmpty() &&
						(flow.size() != mFlowMovingAverage.back()->size() || flow.type() != mFlowMovingAverage.back()->type()))
					{
						mFlowMovingAverage.clear();
					}
					flow = mFlowMovingAverage.update(flow);
				}
				{
					UniqueLock lock(mMutex);
					mRawFlow = rawFlow;
					mFlow = flow;
				}
				mFlowTimestamp = timestamp;
			}
			else
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			mPrevResizedBodyIndex = blurred;
		}
		else
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}
	}
	cout << "Optical flow thread completing." << endl;
}

cv::Mat OpticalFlow::getFlow() const
{
	SharedLock lock(mMutex);
	return mFlow;
}


cv::Mat OpticalFlow::getRawFlow() const
{
	SharedLock lock(mMutex);
	return mFlow;
}


void OpticalFlow::drawFlow(cv::Mat const& flowMat) const
{
	gl::pushModelView();
	glPushAttrib(GL_LINE_WIDTH);
	glDisable(GL_TEXTURE_2D);
	glLineWidth(0.5);
	gInput->setMatricesKinect(fromOcv(flowMat.size()), true);
	//gl::setMatricesWindow(fromOcv(mFlow.size()));
	if (flowMat.type() == CV_32FC2)
	{
		int averageWindowSize = 1;
		float m = 0;
		for (int y = 0; y < flowMat.rows; y += averageWindowSize)
		{
			for (int x = 0; x < flowMat.cols; x += averageWindowSize)
			{
				cv::Scalar mean = cv::mean(flowMat(
					cv::Range(y, min(flowMat.rows, y + averageWindowSize)),
					cv::Range(x, min(flowMat.cols, x + averageWindowSize))));
				float brightness = mean.dot(mean) / (averageWindowSize*averageWindowSize);
				m = max(m, brightness);
				brightness = 0.5f + 0.5f * max(0.f, min(1.f, brightness));
				gl::color(ColorA(brightness, brightness, brightness, 1.0));
				ci::gl::drawLine(Vec2f(x, y), Vec2f(x, y) + Vec2f(mean[0], mean[1]) * mFlowDrawScale);
			}
		}
		//cout << "max flow " << m << endl;
	}
	glPopAttrib();
	gl::popModelView();
}


void OpticalFlow::draw()
{
	if (mDrawRawFlow)
	{
		SharedLock lock(mMutex);
		drawFlow(mRawFlow);
	}
	if (mDrawFlow)
	{
		SharedLock lock(mMutex);
		drawFlow(mFlow);
	}
}