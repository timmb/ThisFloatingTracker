#pragma once

#include "utils/MovingAverage.h"
#include <string>
#include <vector>


/// Filter to extract a smooth expressive control from a single parameter (usually acceleration magnitude)
/// based on the hand control from Impossible Alone (Tim Murray-Browne, 2011).
class DynamicControlCalculator
{
public:
	/// Pass in pointers to variables for these parameters.
	DynamicControlCalculator(std::shared_ptr<float> inputDecay, std::shared_ptr<float> peakDecay, std::shared_ptr<int> movingAverageWindowSize, std::shared_ptr<float> scale);

	void update(float newValue);

	float value;

private:
	// PARAMS
	std::shared_ptr<float> mInputDecay;
	std::shared_ptr<float> mPeakDecay;
	std::shared_ptr<int> mMovingAverageWindowSize;
	std::shared_ptr<float> mScale;

	// VARS
	MovingAverage<float> mMovingAverage;
	float mPeak;
	float mSmoothedInput;
};