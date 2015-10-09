#pragma once
#include "cinder/Vector.h"
#include "cinder/MayaCamUI.h"
#include <set>
#include <atomic>

class ShaderPipeline;
class MidiSender;
class SkylinesPerlinMapper;

struct Globals
{
	float time;
	std::atomic<float> dt;
	// mouse x, y normalized to 0,1
	float mx, my;
	// mouse delta
	float dmx, dmy;
	/// mouse buttons
	bool mouseLeft, mouseMiddle, mouseRight;
	float mouseWheel;
	ci::Vec2i mouseWindowCoordinates;
	ci::Vec2i mouseDeltaWindowCoordinates;
	bool shiftDown, ctrlDown, altDown;
	/// Set of Cinder KeyEvent codes
	std::set<int> keysPressed;
	/// Slightly more thread-safe copy of app::getWindowSize() that survive the window being closed.
	ci::Vec2i windowSize;
	//std::atomic<float> mUserXPos;
	bool isTrackingUser;

	/// Press tab to toggle, backspace to reset view when enabled
	bool enable3dView;
	ci::MayaCamUI cam3d;

	/// Shader pipeline for use by sketches. This retains a copy of original image
	std::unique_ptr<ShaderPipeline> shaderPipeline;

	// Allow something to take exclusive control of keyboard/mouse input
	enum ExclusiveController
	{
		NONE = 0,
		KINECT_ALIGNER,
	};
	ExclusiveController exclusiveController;

	Globals();
};

extern Globals g;
