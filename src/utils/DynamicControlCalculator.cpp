#include "DynamicControlCalculator.h"
#include <limits>
#include "Common.h"

using namespace std;

DynamicControlCalculator::DynamicControlCalculator(std::shared_ptr<float> inputDecay, std::shared_ptr<float> peakDecay, std::shared_ptr<int> movingAverageWindowSize, std::shared_ptr<float> scale)
	: value(0)
	, mInputDecay(std::move(inputDecay))
	, mPeakDecay(std::move(peakDecay))
	, mMovingAverageWindowSize(std::move(movingAverageWindowSize))
	, mScale(std::move(scale))
	, mPeak(0)
	, mSmoothedInput(0)
{
	assert(mInputDecay && mPeakDecay && mMovingAverageWindowSize && mScale);
}

void DynamicControlCalculator::update(float input)
{
	*mMovingAverageWindowSize = max(1, *mMovingAverageWindowSize);
	mMovingAverage.mNumSamples = *mMovingAverageWindowSize;

	input = mMovingAverage.update(input);
	mSmoothedInput += *mInputDecay * (input - mSmoothedInput);
	// filter out bad values
	if (mSmoothedInput != mSmoothedInput || abs(mSmoothedInput) == numeric_limits<float>::infinity())
	{
		mSmoothedInput = input;
	}

	mPeak = max(input, *mPeakDecay * mPeak);

	float a = 0.19375f;
	float b = 0.796875f;

	value = a * input + (1.f - a) * mSmoothedInput;
	value = b * value + (1.f - b) * mPeak;
	value *= *mScale;
	value = clamp(value);


	//for (int i = handSpeedAverageWindowSize - 1; i>0; --i)
	//	handSpeeds[h][i] = handSpeeds[h][i - 1];
	//handSpeeds[h][0] = getJoint(h == 0 ? ::XN_SKEL_LEFT_HAND : ::XN_SKEL_RIGHT_HAND).mVel.length();
	//handSpeed[h] = 0;
	//for (int i = 0; i<handSpeedAverageWindowSize; ++i)
	//	handSpeed[h] += handSpeeds[h][i];
	//handSpeed[h] /= handSpeedAverageWindowSize;
	//handSpeed[h] *= 0.453*.001;
	//handSmoothSpeed[h] += (1.f - jointParameters.expressionDecay)*framerateFactor * (handSpeed[h] - handSmoothSpeed[h]);
	//float peakDecay = 1. - framerateFactor*(1. - 0.965);
	//handPeakSpeed[h] *= peakDecay;
	//handPeakSpeed[h] = max<double>(handPeakSpeed[h], handSpeed[h]);
	//float a = 0.19375;
	//handExpression[h] = a*handSpeed[h] + (1. - a)*handSmoothSpeed[h];
	//float b = 0.796875;//0.823438;
	//handExpression[h] = b*handExpression[h] + (1. - b)*handPeakSpeed[h];
	//handExpression[h] = max(0.f, min(1.f, handExpression[h]));
	//handExpressionSlow[h] = handPeakSpeed[h];
}