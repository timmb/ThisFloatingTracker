#pragma once


#include "Common.h"
#include "MidiOut.h"
#include <boost/thread.hpp>

class MidiSender
{
public:
	MidiSender();
	virtual ~MidiSender();

	void setup();
	void stop();

	// **** Replace these with update functions for your own sketch parameters

	/// \param heiroglyphProgress is how far through all the heiroglyphs we are.
	void updateDialogue(float velocity, float roughVelocity, float accel, float peakHeight, bool isInHeiroglyph, float heiroglyphProgress, float plantDensity, float plantGrowthDensity, bool isInTraces);

	void updateSkylines(float qomForAmpEffect);

	// *********

private:
	void refreshMidiPorts();
	/// mutex on thread must be locked when calling this
	void p_updateMidiPortFromName();

	float mSmoothingAmount;
	/// Some parameters use a different 'slow' smoothing amount
	float mSlowSmoothingAmount;

	/// Stretch values away from 0.5 towards the limits (0.0 or 1.0)
	float mStretchVelocity;
	float mStretchRoughVelocity;
	float mStretchAccel;
	float mStretchPeakHeight;
	float mStretchPlantDensity;
	float mStretchPlantGrowthDensity;
	float mStretchUserPan;
	float mStretchAmpEffect;
	float mOffsetAmpEffect;

	int mMidiPortNum;

	// canonical
	std::string mMidiPortName;
	std::vector<std::string> mMidiPortNameList;

	std::shared_ptr<Parameter<int>> mMidiPortNameParameter;

	class Thread
	{
	public:
		Thread();
		virtual ~Thread();

		// *** Replace these with your own parameters for all sketches
		void setState(
			float velocity, float roughVelocity, float accel, float peakHeight, 
			bool isInHeiroglyph, float heiroglyphProgress, float plantDensity, float plantGrowthDensity, 
			bool isInTraces, float userPan, float ampEffect);

		/// blocks
		void stop() { mIsRunning = false; }

		ci::midi::MidiOut mMidiOut;
		/// guards everything including mMidiOut. do not call setDialogueState while it's locked
		boost::mutex mMutex;

	private:
		void run();

		// previousState for heiroglyphs
		bool mWasInHeiroglyph;
		bool mWerePlantsGrowing;

		// ** Include a current value for all of your parameters.
		float mCurrVelocity;
		float mCurrRoughVelocity;
		float mCurrAccel;
		float mCurrPeakHeight;
		float mCurrIsInHeiroglyph;
		float mCurrHeiroglyphProgress;
		float mCurrPlantDensity;
		float mCurrPlantGrowthDensity;
		float mCurrIsInTraces;
		float mCurrUserPan;
		float mCurrAmpEffect;

		// ** Include a target state for all your parameters. The thread runs faster than the Cinder
		// event loop so it can interpolate smoothly to these values
		float mTargetVelocity;
		float mTargetRoughVelocity;
		float mTargetAccel;
		float mTargetPeakHeight;
		float mTargetIsInHeiroglyph;
		float mTargetHeiroglyphProgress;
		float mTargetPlantDensity;
		float mTargetPlantGrowthDensity;
		float mTargetIsInTraces;
		float mTargetUserPan;
		float mTargetAmpEffect;

		boost::thread mThread;
		std::atomic<bool> mIsRunning;
	};
	Thread mThread;
};