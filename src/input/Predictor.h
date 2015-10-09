/*
 *  Predictor.h
 *  HarmonicMotion
 *
 *  Based on Impossible Alone, created by Tim Murray-Browne on 28/9/2011.
 *  Copyright 2013 Tim Murray-Browne. All rights reserved.
 *
 */



#pragma once

#include <vector>

/// Double exponential estimator. Provide a value and it will give an estimate
/// m-1 steps in the future by applying a trend estimated using a simple
/// exponential (IIR) filter.
/// 
/// Based on equations in paper: P. Kalekar, “Time series forecasting using
/// holt-winters exponential smoothing,” tech. rep., Kanwal Rekhi School of
/// Information Technology, Mumbai, India, 2004.
template<class T, class Scalar=double>
class Predictor
{
public:
	/// See explanation below of parameters
	Predictor(int predictionSize=1, Scalar alpha=0.5f, Scalar beta=0.5f);
	
	T const& update(T newInput); // Returns a reference to the final (m-1'th) forecasted value
	
	/// Resets the Predictor to as if it had just been constructed, except
	/// its initial value is set to x rather than T(). The initial
	/// speed estimate is set to x * 0.
	void reset(T const& x);
	
	/// NB. How many steps into the future to predict. NB this includes an estimate of the current value. So if you want
	/// the value k steps into the future you'll need to set predictionSize = k+1
	int predictionSize;
	/// Amount of first-order smoothing (values between 0 and 1)
	Scalar alpha; // higher values -> more smoothing
	/// Amount of second-order smoothing (values between 0 and 1)
	Scalar beta;
	/// Amount that the trend estimation decays each update (predicting things slow down naturally)
	Scalar trendDecay;
	
	/// \return a vector of size predictionSize of values t=0 upto t=predictionSize-1
	std::vector<T> const& prediction() const;

private:
	std::vector<T> mPrediction; // estimate of next m steps including the current estimate: forecast[i] is estimate of time t+i where t is current time
						// So forecast[0] is estimate of the current value (S_t)
//	T s; // copy of forecast[0] to ensure it is not changed
	
	T b; // estimate of the current trend
	
	bool s_initialized; // Set to true once the first input has been received (the `s` refers to forecast[0])
	bool b_initialized; // Set to true once the second input has been received
};





template<class T, class Scalar>
Predictor<T, Scalar>::Predictor(int predictionSize_, Scalar alpha_, Scalar beta_)
: predictionSize(predictionSize_)
, alpha(alpha_)
, beta(beta_)
, trendDecay(1.f)
, mPrediction(predictionSize_)
, b(T())
, s_initialized(false)
, b_initialized(false)
//, s(T())
{
}

template<class T, class Scalar>
void Predictor<T,Scalar>::reset(T const& x)
{
	s_initialized = false;
	b_initialized = false;
	b = x * 0.;
	mPrediction.assign(predictionSize, x);
}


template<class T, class Scalar>
std::vector<T> const& Predictor<T,Scalar>::prediction() const
{
	return mPrediction;
}





template<class T, class Scalar>
T const& Predictor<T,Scalar>::update(T newInput)
{
	predictionSize = std::max(1, predictionSize);
	if (mPrediction.size() != predictionSize)
	{
		mPrediction.resize(predictionSize);
	}
	assert(mPrediction.size() == predictionSize);
	
	if (s_initialized && b_initialized) // Normal update
	{
			// Calculate new estimate. forecast[0] == S_(t-1)
		T newEstimate( (Scalar(1) - alpha) * newInput + alpha * (mPrediction[0] + b) );
			// Update b:
		b *= beta;
		b += (Scalar(1) - beta) * (newEstimate - mPrediction[0]);
		b *= trendDecay;
			// Set forecast[0] = S_t
		mPrediction[0] = newEstimate;
	}
	else if (!s_initialized)
	{
		mPrediction[0] = newInput;
		s_initialized = true;
		// b hasn't been set but the constructor initialises it to T() (i.e. zero)
	}
	else
	{
		assert(!b_initialized);
		b = newInput - mPrediction[0];
		mPrediction[0] *= alpha;
		mPrediction[0] += (1. - alpha) * newInput;
		b_initialized = true;
	}
	
		// Fill out the forecast using b as a linear trend
	for (int i=1; i<predictionSize; i++)
	{
		mPrediction[i] = mPrediction[i-1] + b;
	}
	
//	s = mPrediction[0]; // keep a copy of forecast[0] in case it is edited
	
	return mPrediction[predictionSize-1];
}


