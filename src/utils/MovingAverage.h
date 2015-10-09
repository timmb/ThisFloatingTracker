#pragma once

#include <boost/circular_buffer.hpp>
#include "CinderOpenCV.h"

template <typename T>
class MovingAverage
{
public:
	MovingAverage();
	T update(T const& newValue);
	void clear();
	/// may be nullptr
	T* back();
	bool isEmpty() const;

	int mNumSamples;

private:
	/// Sum of data in mBuffer
	T mCurrentSum;
	boost::circular_buffer<T> mBuffer;

	T mCurrentRecalculationSum;
	// how many vaues stored in current recalculation sum
	int mCurrentRecalculationSumCount;
};

template <typename T>
T clone(T const& rhs)
{
	return rhs;
}

template <>
inline
cv::Mat clone(cv::Mat const& rhs)
{
	return rhs.clone();
}

template <typename T>
MovingAverage<T>::MovingAverage()
: mNumSamples(40)
, mBuffer(40)
, mCurrentSum(T())
, mCurrentRecalculationSum(T())
, mCurrentRecalculationSumCount(0)
{}

template <typename T>
bool MovingAverage<T>::isEmpty() const
{
	return mBuffer.empty();
}

template <typename T>
T* MovingAverage<T>::back()
{
	return isEmpty() ? nullptr : &mBuffer.back();
}

template <typename T>
void MovingAverage<T>::clear()
{
	mBuffer.clear();
	mCurrentRecalculationSum = T();
	mCurrentRecalculationSumCount = 0;
}

template <typename T>
T MovingAverage<T>::update(T const& d)
{
	if (mNumSamples != mBuffer.capacity())
	{
		mBuffer.set_capacity(mNumSamples);
	}

	if (mBuffer.empty())
	{
		mCurrentSum = clone(d);
	}
	else
	{
		mCurrentSum += d;
		if (mBuffer.full())
		{
			mCurrentSum = mCurrentSum - mBuffer.front();
			mBuffer.pop_front();
		}
	}
	mBuffer.push_back(d);
	// Also incrementally recalculate to get rid of rounding
	// errors.
	if (mCurrentRecalculationSumCount == 0)
	{
		mCurrentRecalculationSum = clone(d);
	}
	else
	{
		mCurrentRecalculationSum += d;
	}
	mCurrentRecalculationSumCount++;
	if (mCurrentRecalculationSumCount == mNumSamples && mBuffer.size() == mNumSamples)
	{
		mCurrentSum = mCurrentRecalculationSum;
	}
	if (mCurrentRecalculationSumCount >= mNumSamples)
	{
		mCurrentRecalculationSum = T();
		mCurrentRecalculationSumCount = 0;
	}
	return mCurrentSum / mBuffer.size();
}