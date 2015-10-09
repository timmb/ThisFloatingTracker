#include "MidiSender.h"
#include "Globals.h"

typedef boost::lock_guard<boost::mutex> Lock;

using namespace std;

namespace
{
	const int DIALOGUE_NOTE = 60;
	std::atomic<float> smoothingAmount = 0.5f;
	std::atomic<float> slowSmoothingAmount = 0.5f;
}

MidiSender::MidiSender()
	: mMidiPortName("loopMIDI Port")
	, mSmoothingAmount(0.5f)
{
	pars.addButton([this] { refreshMidiPorts(); }, "Refresh port list", { "MidiSender" });
#define PARAM(var, init) var = init; pars.addParam(&var, #var, { "MidiSender" }, "");
#define OPTION(var, init) var = init; pars.addParam(&var, #var, { "MidiSender" }, "unsaved");

	auto p = pars.addParam(&mSmoothingAmount, "Smoothing amount", { "MidiSender" }, "");
	p->customCallback = [this]{ 
		smoothingAmount = mSmoothingAmount; 
	};
	p = pars.addParam(&mSlowSmoothingAmount, "Slow smoothing amount", { "MidiSender" }, "");
	p->customCallback = [this]{
		slowSmoothingAmount = mSlowSmoothingAmount;
	};
	refreshMidiPorts();
	PARAM(mStretchVelocity, 1.f);
	PARAM(mStretchRoughVelocity, 1.f);
	PARAM(mStretchAccel, 1.f);
	PARAM(mStretchPeakHeight, 1.f);
	PARAM(mStretchPlantDensity, 1.f);
	PARAM(mStretchPlantGrowthDensity, 1.f);
	PARAM(mStretchUserPan, 1.f);
	PARAM(mStretchAmpEffect, 1.f);
	PARAM(mOffsetAmpEffect, 0.f);
}

MidiSender::~MidiSender()
{
}

void MidiSender::setup()
{
	smoothingAmount = mSmoothingAmount;
}

void MidiSender::stop()
{
	mThread.stop();
}

void MidiSender::refreshMidiPorts()
{
	Lock lock(mThread.mMutex);

	if (mMidiPortNameParameter != nullptr)
	{
		pars.removeParam(mMidiPortNameParameter);
	}
	mMidiPortNameList = { "None" };
	for (std::string s : mThread.mMidiOut.getPortList())
	{
		mMidiPortNameList.push_back(std::move(s));
	}
	mMidiPortNameParameter = shared_ptr<Parameter<int>>(new Parameter<int>(&mMidiPortNum, "Port", { "MidiSender" }, "unsaved", [this] {
		// nb we've added 'none' to front of list
		if (!(0 <= mMidiPortNum && mMidiPortNum < mMidiPortNameList.size()))
		{
			mMidiPortNum = 0;
		}
		mMidiPortName = mMidiPortNameList.at(mMidiPortNum);
		Lock lock(mThread.mMutex);
		p_updateMidiPortFromName();
	}));
	mMidiPortNameParameter->enumerationLabels = mMidiPortNameList;
	pars.addParam(mMidiPortNameParameter);
	p_updateMidiPortFromName();
}

void MidiSender::p_updateMidiPortFromName()
{
	int index = find(begin(mMidiPortNameList), end(mMidiPortNameList), mMidiPortName) - begin(mMidiPortNameList);
	if (index == 0)
	{
		mThread.mMidiOut.closePort();
	}
	else if (index < mMidiPortNameList.size())
	{
		mThread.mMidiOut.openPort(index - 1);
	}
}


inline
int toMidi(float f)
{
	// add a little bit on so that values close to 1. hit 127
	return max(0, min(127, int(f * 127.f + 0.5f)));
}

void MidiSender::updateSkylines(float qom)
{
	qom = clamp(0.5f + (qom - 0.5f - mOffsetAmpEffect)*mStretchAmpEffect/(1.f-mOffsetAmpEffect));
	// When the skylines sketch was running, this are the MIDI Values for all parameters that get sent
	mThread.setState(0, 0, 0, 0, false, 0, 0, 0, false, 0, qom);
}

void MidiSender::updateDialogue(float velocity, float roughVelocity, float accel, float peakHeight, bool isInHeiroglyph, float heiroglyphProgress, float plantDensity, float plantGrowthDensity, bool isInTraces)
{
	velocity = clamp(0.5f + (velocity - 0.5f) * mStretchVelocity);
	roughVelocity = clamp(0.5f + (roughVelocity - 0.5f) * mStretchRoughVelocity);
	accel = clamp(0.5f + (accel - 0.5f) * mStretchAccel);
	peakHeight = clamp(0.5f + (peakHeight - 0.5f) * mStretchPeakHeight);
	plantDensity = clamp(0.5f +(plantDensity - 0.5f) * mStretchPlantDensity);
	plantGrowthDensity = clamp(0.5f + (plantGrowthDensity - 0.5f) * mStretchPlantGrowthDensity);

	//float userPan = g.mUserXPos * mStretchUserPan + 0.5f;
	float userPan = 0.5f;

	// When the dialogue sketch was running, this are the MIDI Values for all parameters that get sent
	mThread.setState(velocity, roughVelocity, accel, peakHeight, isInHeiroglyph, heiroglyphProgress,
		plantDensity, plantGrowthDensity, isInTraces, userPan, 0);

}

MidiSender::Thread::Thread()
	: mWasInHeiroglyph(false)
	, mWerePlantsGrowing(false)
	, mCurrVelocity(0)
	, mCurrAccel(0)
	, mCurrRoughVelocity(0)
	, mCurrPeakHeight(0)
	, mCurrIsInHeiroglyph(0)
	, mCurrHeiroglyphProgress(0)
	, mCurrIsInTraces(0)
	, mCurrUserPan(0.5f)
	, mCurrAmpEffect(0)
	, mTargetVelocity(0)
	, mTargetAccel(0)
	, mTargetRoughVelocity(0)
	, mTargetPeakHeight(0)
	, mTargetIsInHeiroglyph(0)
	, mTargetHeiroglyphProgress(0)
	, mTargetIsInTraces(0)
	, mTargetUserPan(0.5f)
	, mTargetAmpEffect(0)
	, mIsRunning(false)
{
	mThread = boost::thread([this] { run(); });
}

MidiSender::Thread::~Thread()
{
	mIsRunning = false;
	mThread.interrupt();
	mThread.join();
}

// ** Replace with your own parameters
void MidiSender::Thread::setState(float velocity, float roughVelocity, float accel, float peakHeight, bool isInHeiroglyph, 
	float heiroglyphProgress, float plantDensity, float plantGrowthDensity, bool isInTraces, float userPan, float ampEffect)
{
	Lock lock(mMutex);
	mTargetVelocity = velocity;
	mTargetRoughVelocity = roughVelocity;
	mTargetAccel = accel;
	mTargetIsInHeiroglyph = isInHeiroglyph;
	mTargetHeiroglyphProgress = heiroglyphProgress;
	mTargetPeakHeight = peakHeight;
	mTargetPlantDensity = plantDensity;
	mTargetPlantGrowthDensity = plantGrowthDensity;
	mTargetIsInTraces = isInTraces;
	mTargetUserPan = userPan;
	mTargetAmpEffect = ampEffect;
}

namespace
{
	float const epsilon = 0.0001f;

	inline
	void smooth(float& currVal, float targetVal, float smoothingAmount)
	{
		assert(targetVal == targetVal);
		currVal += min(1.f, smoothingAmount * g.dt) * (targetVal - currVal);
		currVal = abs(targetVal - currVal) < epsilon ? targetVal : currVal;
		assert(currVal == currVal);
		if (currVal != currVal)
		{
			currVal = 0;
		}
	}
}

void MidiSender::Thread::run()
{
	mIsRunning = true;
	while (mIsRunning)
	{
		{
			Lock lock(mMutex);


			if (mMidiOut.isOpen())
			{
				try
				{
					// ** REMOVE AND REPLACE WITH YOUR OWN PARAMETERS

					// Custom midi logic for dialogue sketch to trigger note events.

					if (mTargetIsInHeiroglyph < epsilon && mWasInHeiroglyph)
					{
						mMidiOut.sendNoteOff(10, DIALOGUE_NOTE);
						mWasInHeiroglyph = false;
					}

					smooth(mCurrVelocity, mTargetVelocity, smoothingAmount);
					smooth(mCurrRoughVelocity, mTargetRoughVelocity, smoothingAmount);
					smooth(mCurrAccel, mTargetAccel, smoothingAmount);
					smooth(mCurrPeakHeight, mTargetPeakHeight, smoothingAmount);
					smooth(mCurrHeiroglyphProgress, mTargetHeiroglyphProgress, smoothingAmount);
					smooth(mCurrIsInHeiroglyph, mTargetIsInHeiroglyph, smoothingAmount);
					smooth(mCurrPlantDensity, mTargetPlantDensity, smoothingAmount);
					smooth(mCurrPlantGrowthDensity, mTargetPlantGrowthDensity, smoothingAmount);
					smooth(mCurrIsInTraces, mTargetIsInTraces, slowSmoothingAmount);
					smooth(mCurrUserPan, mTargetUserPan, smoothingAmount);
					smooth(mCurrAmpEffect, mTargetAmpEffect, smoothingAmount);

					mMidiOut.sendControlChange(10, 0, toMidi(mCurrVelocity));
					mMidiOut.sendControlChange(10, 1, toMidi(mCurrHeiroglyphProgress));
					mMidiOut.sendControlChange(10, 2, toMidi(mCurrHeiroglyphProgress * 2));
					mMidiOut.sendControlChange(10, 3, toMidi(mCurrHeiroglyphProgress * 4));
					mMidiOut.sendControlChange(10, 4, toMidi(mCurrIsInHeiroglyph));
					mMidiOut.sendControlChange(10, 5, toMidi(mCurrVelocity * mCurrIsInHeiroglyph));
					mMidiOut.sendControlChange(10, 6, toMidi(mCurrRoughVelocity));
					mMidiOut.sendControlChange(10, 7, toMidi(mCurrAccel));
					mMidiOut.sendControlChange(10, 8, toMidi(mCurrPeakHeight));
					mMidiOut.sendControlChange(10, 9, toMidi(mCurrPlantDensity));
					mMidiOut.sendControlChange(10, 10, toMidi(mCurrPlantGrowthDensity));
					mMidiOut.sendControlChange(10, 11, toMidi(mCurrIsInTraces));
					mMidiOut.sendControlChange(10, 12, toMidi(mCurrUserPan));
					mMidiOut.sendControlChange(10, 13, toMidi(mCurrAmpEffect));

					if (mTargetIsInHeiroglyph >= epsilon && !mWasInHeiroglyph)
					{
						mMidiOut.sendControlChange(10, 14, 127);
						mMidiOut.sendNoteOn(10, DIALOGUE_NOTE, 127);
						mMidiOut.sendControlChange(10, 14, 0);
						mWasInHeiroglyph = true;
					}
					bool arePlantsGrowing = mTargetPlantGrowthDensity > epsilon;
					if (arePlantsGrowing && !mWerePlantsGrowing)
					{
						mMidiOut.sendNoteOn(10, 58);
						mMidiOut.sendNoteOff(10, 58);
						mWerePlantsGrowing = true;
					}
					mWerePlantsGrowing = arePlantsGrowing;
				}
				catch (RtError & e)
				{
					cerr << "[MidiSender] RtError: ";
					e.printMessage();
				}
			}
			else
			{
				mWasInHeiroglyph = false;
				mWerePlantsGrowing = true;
			}
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(15));
	}
	if (mMidiOut.isOpen())
	{
		mMidiOut.sendControlChange(10, 0, 0);
		mMidiOut.sendControlChange(10, 1, 0);
		mMidiOut.sendControlChange(10, 2, 0);
		mMidiOut.sendControlChange(10, 3, 0);
		mMidiOut.sendControlChange(10, 4, 0);
		mMidiOut.sendControlChange(10, 5, 0);
		mMidiOut.sendControlChange(10, 6, 0);
		mMidiOut.sendControlChange(10, 7, 0);
		mMidiOut.sendControlChange(10, 8, 0);
		mMidiOut.sendControlChange(10, 9, 0);
		mMidiOut.sendControlChange(10, 10, 0);
		mMidiOut.sendControlChange(10, 12, 0);
		mMidiOut.sendControlChange(10, 13, 0);

		if (mWasInHeiroglyph)
		{
			mMidiOut.sendNoteOff(10, DIALOGUE_NOTE);
		}
	}
}