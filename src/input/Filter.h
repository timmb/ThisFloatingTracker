//
//  Smoother
//  HarmonicMotion
//
//  Created by Tim Murray-Browne on 24/10/2013.
//  Copyright (c) 2013 Tim Murray-Browne. All rights reserved.
//

#pragma once

// We use the term Filter to describe an object that processes a noisy signal
// and provides an estimate of its current value


#include <deque>
#include <vector>
#include "input/Predictor.h"


	
	/// Savitzky-Golay smoothing filter applied to values from a Predictor.
	/// Based on coefficients from p651 of NUMERICAL RECIPES IN C: THE ART OF SCIENTIFIC COMPUTING (ISBN 0-521-43108-5) for the 4th order 11 point set
	/// and from http://www.statistics4u.info/fundstat_eng/cc_savgol_coeff.html for the cubic sets
	template<typename T, typename Scalar=float, typename PredictorType=Predictor<Scalar>>
	class FilterSavitzky
	{
	public:
		/// The number of Savitzky-Golay coefficients we are using (and the
		/// size of the sample used by the filter)
		static const int LENGTH = 15;
		
		FilterSavitzky();
		
		/// \return a reference to our estimate of the current value or x if not
		/// enough values havebeen received yet. This reference may be invalidated
		/// the next time update() is called.
		T const& update(T const& x);
		
		/// Resets the memory of the filter to as if it had just been created,
		/// but setting its starting value to x
		void reset(T const& x);
		
		Predictor<T, Scalar> predictor;
		
		
	private:
		/// Updates mValues with x using predictor with the outcome that
		/// mValues[LENGTH/2]==x
		void pushBack(T const& x);
		/// Applies the Savitsky-Golay filter to update mFilteredValue
		void applyFilter();
		std::deque<T> mValues;
		std::vector<Scalar> mCoefficients;
		/// This is the filtered value of the last value (which is kept
		/// in mValues[LENGTH/2]
		T mFilteredValue;
		
		
	};
	
	
	template<typename T, typename Scalar, typename PredictorType>
	FilterSavitzky<T,Scalar,PredictorType>::FilterSavitzky()
	: predictor(LENGTH/2 + 1) 	// + 1 so that we are looking SAVITSKY_LENGTH/2 into the future
	, mFilteredValue(T())
	{
		//	coeff = { 0.042, -0.105, -0.023, 0.140, 0.280, 0.333, 0.280, 0.140, -0.023, -0.105, 0.042 }; // 4th order 11 points
		mCoefficients = {
			Scalar(-0.070588235294117646), Scalar(-0.011764705882352941), Scalar(0.038009049773755653),  Scalar(0.078733031674208143),  Scalar(0.11040723981900452),
			Scalar(0.1330316742081448),    Scalar(0.14660633484162897),  Scalar(0.151131221719457),     Scalar(0.14660633484162897),   Scalar(0.1330316742081448),
			Scalar(0.11040723981900452),   Scalar(0.078733031674208143), Scalar(0.038009049773755653), Scalar(-0.011764705882352941), Scalar(-0.070588235294117646)
		}; // 3rd order 15 points
		assert(mCoefficients.size() == LENGTH);
	}
	
	
	template <class T, class Scalar, class PredictorType>
	void FilterSavitzky<T,Scalar,PredictorType>::reset(T const& x)
	{
		mFilteredValue = x;
		mValues.clear();
		predictor.reset(x);
	}
	
	
	
	template<class T, class Scalar, class PredictorType>
	T const& FilterSavitzky<T,Scalar,PredictorType>::update(T const& x)
	{
		pushBack(x);
		if (mValues.size() < LENGTH)
		{
			mFilteredValue = x;
			return x;
		}
		applyFilter();
		return mFilteredValue;
	}
	
	
	
	// Let LENGTH = 11. Then
	// INVAR: mValues.size()==LENGTH => (mValues[0..5) == x[-5..0) and mValues[5] == x[0] and mValues[6..11) == est_x[1,6))
	// INVAR2: mValues.size() <= LENGTH/2 or mValues.size()==LENGTH
	template<class T, class Scalar, class PredictorType>
	void FilterSavitzky<T,Scalar,PredictorType>::pushBack(T const& x)
	{
		//	lastInput = x;
		
		assert(mValues.size() <= LENGTH/2 || mValues.size() == LENGTH); // INVAR2
		predictor.update(x);
		std::vector<T> const& prediction = predictor.prediction();
		assert(prediction.size() == LENGTH/2 + 1);
		// We want to copy forecast into values[SAVITSKY_LENGTH/2, SAVITSKY_LENGTH)
		// unless there aren't enough values
		if (mValues.size()==LENGTH)
		{
			// values[0..5] = x[-6..-1]
			mValues.pop_front();                       // values[0..5) = x[-5.. 0) and values.size()==10
			mValues[LENGTH/2] = x;         // values[5] = x[0] => first two clauses of INVAR restored
			int j(1); // counter for prediction
			for (int i = LENGTH/2 + 1; i<LENGTH - 1; i++)
			{ // LENGTH-1 as we have popped an element that needs replaing through a push
				assert(0<=j && j<prediction.size());
				mValues[i] = prediction[j];
				++j;
			}
			// values[6..10) = est_x[1..5)
			mValues.push_back(prediction.at(j)); // values[6..11) = est_x[1..6) => INVAR restored
			
			assert(mValues.size() <= LENGTH/2 || mValues.size() == LENGTH); // INVAR2
		}
		else if (mValues.size() < LENGTH/2)
		{
			mValues.push_back(x); // leave values below requisite size for filtering
			assert(mValues.size() <= LENGTH/2 || mValues.size() == LENGTH); // INVAR2
		}
		else
		{
			// The moment we hit LENGTH/2 then we fill out values to the full length
			assert(mValues.size()==LENGTH/2);
			// values[0..5) = x[-5..0)
			mValues.push_back(x);
			// values[5] = x[0]
			assert(mValues.at(LENGTH/2)==x);
			
			// LENGTH is odd so there are still LENGTH/2 places remaining to be filled
			// prediction is of size LENGTH/2 + 1 as it includes an estimate of the present.
			for (auto it = prediction.cbegin()+1; it!=prediction.cend(); ++it)
			{
				mValues.push_back(*it);
			}
			assert(mValues.size()==LENGTH); // INVAR2 restored
			// values[6..11) = est_x[1..6)
			// INVAR restored
		}
		assert(mValues.size() <= LENGTH/2 || mValues.size() == LENGTH); // INVAR2
	}
	
	
	template<class T, class Scalar, class PredictorType>
	void FilterSavitzky<T,Scalar,PredictorType>::applyFilter()
	{
		assert(mValues.size() == LENGTH);
		assert(mCoefficients.size() == LENGTH);
		
		// Multiply by zero rather than use T() to prevent loss of any
		// metadata
		mFilteredValue *= 0.;
		auto vit = mValues.cbegin();
		auto cit = mCoefficients.cbegin();
		for (; cit!=mCoefficients.end(); ++vit, ++cit)
		{
			mFilteredValue += *cit * *vit;
		}
	}
