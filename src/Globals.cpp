#include "Globals.h"
#include "ShaderPipeline.h"

Globals::Globals()
	: time(-0.04)
	, dt(0.04)
	, mx(0)
	, my(0)
	, dmx(0)
	, dmy(0)
	, mouseLeft(false)
	, mouseMiddle(false)
	, mouseRight(false)
	, mouseWheel(0.f)
	, shiftDown(false)
	, ctrlDown(false)
	, altDown(false)
	, enable3dView(false)
	, shaderPipeline(new ShaderPipeline(true))
	, exclusiveController(NONE)
	, isTrackingUser(false)
{}

Globals g;


