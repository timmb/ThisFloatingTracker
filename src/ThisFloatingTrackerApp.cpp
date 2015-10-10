#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "ShaderPipeline.h"
#include "Common.h"
#include "OscReceiver.h"
#include "cinder/Font.h"
#include "cinder/gl/Fbo.h"
#include "cinder/MayaCamUI.h"
#include "CinderOpenCv.h"
#include "Globals.h"
#include "cinder/Easing.h"
#include "utils/Profiler.h"
#include "utils/MovingTrough.h"
#include "OscTransmitter.h"
#include "Sketch.h"
#include "Staircase.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class ThisFloatingTrackerApp : public AppNative {
public:
	ThisFloatingTrackerApp();
	virtual void prepareSettings(Settings* settings) override;
	virtual void setup() override;
	virtual void mouseUp(MouseEvent event) override;
	virtual void mouseDown(MouseEvent event) override;
	virtual void mouseWheel(MouseEvent event) override;
	virtual void mouseMove(MouseEvent event) override;
	virtual void mouseDrag(MouseEvent event) override;
	virtual void keyDown(KeyEvent event) override;
	virtual void keyUp(KeyEvent event) override;
	virtual void update() override;
	virtual void draw() override;
	virtual void resize() override;
	virtual void shutdown() override;

private:
	void checkRenderTargetIsCorrectSize();
	void updateMouseGlobals(MouseEvent event);
	void updateScene();
	void crossfadeToSketch(int newSketch);
	void render();
	void profiledRender();
	void mySetFullScreen(bool isFullScreen);
	void setDebugDisplay(bool isDebugDisplay);


	ShaderPipeline mShaderPipeline;
	vector<shared_ptr<Sketch>> mSketches;
	int mSelectedSketch;
	OscReceiver mOsc;
	OscTransmitter mOscTransmitter;
	//MidiMappingHelper mMidiMappingHelper;
	//MidiReceiver mMidiReceiver;

	float mFps;

	/// export image of each frame.
	bool mEnableRecorder;

	//Vec2i mRenderTargetSize;

	// --- Profiling and debug rendering
	bool mDoPerformanceTestOnNextFrame;
	bool mDrawFps;
	MovingAverage<float> mFpsAverage;
	float mFpsAverageLast;
	MovingTrough<float> mFpsTrough;
	float mFpsTroughLast;
	bool mIsFullScreen;
	Vec2i mNonFullScreenWindowPos;
	Vec2i mNonFullScreenWindowSize;
	bool mEnableDebugDisplay;

	// Touch OSC vars
	float mFadeValue;
	float mFadeTarget;
	float mNormalizedFadeDuration;
	float mMaxFadeDuration;

	// to test keyboard and tablet are working
	bool mDummyToggle;
	// for tablet
	float mNormalizedFps;

	// Sketch crossfading
	bool mIsCrossfading;
	float mCrossfadeTargetSketch;

};

ThisFloatingTrackerApp::ThisFloatingTrackerApp()
	: mSelectedSketch(0)
	, mOsc(2324)
	, mFps(-42)
	, mEnableRecorder(false)
	, mFadeValue(1.f)
	, mFadeTarget(1.f)
	, mNormalizedFadeDuration(0.3f)
	, mDoPerformanceTestOnNextFrame(false)
	, mDrawFps(false)
	, mFpsAverageLast(-42)
	, mFpsTroughLast(-42)
	, mIsFullScreen(false)
	, mDummyToggle(true)
	, mNormalizedFps(0)
	, mIsCrossfading(false)
	, mCrossfadeTargetSketch(0)
{

	gAspectRatio = float(gRenderTargetSize.x) / gRenderTargetSize.y;
}

void ThisFloatingTrackerApp::shutdown()
{
	gInput->stop();
	// trigger destructor on global params before the window is destroyed
	pars.save();
	pars.clear();
}

using namespace ci::gl;

void ThisFloatingTrackerApp::prepareSettings(Settings* settings)
{
#ifdef CINDER_MSW
	settings->enableConsoleWindow();
#endif
	settings->setWindowSize(1128, 420);
}

void ThisFloatingTrackerApp::setup()
{
#if !(defined(DEBUG) || defined(_DEBUG))
	//mySetFullScreen(true);
#endif

	// fix slow cout
	setvbuf(stdout, 0, _IOLBF, 4096);

	gDebugFont = gl::TextureFont::create(ci::Font("Arial", 10));
	gDebugFontMedium = gl::TextureFont::create(ci::Font("Arial", 20));
	gDebugFontBig = gl::TextureFont::create(ci::Font("Arial", 50));
	g.windowSize = app::getWindowSize();

	int majorVersion(-42), minorVersion(-42);
	glGetIntegerv(GL_MAJOR_VERSION, &majorVersion);
	glGetIntegerv(GL_MINOR_VERSION, &minorVersion);
	cout << "OpenGL version: " << majorVersion << "." << minorVersion << endl;

	pars.addParam(new Parameter<int>(&mSelectedSketch, "Selected sketch"));
	pars.addParam(new Parameter<float>(&mFadeValue, "Fade out", { "Show" }, "unsaved"));
	pars.addParam(new Parameter<float>(&mNormalizedFadeDuration, "Fade change duration", { "Show" }));
	pars.addParam(new Parameter<float>(&mMaxFadeDuration, "Fade change max duration", { "Show" }));

	pars.load(getAssetPath("params/params.json").string());

	pars.addParam(&mEnableDebugDisplay, "Debug display", { "Rendering" });
	pars.addParam(new Parameter<int>(&gRenderTargetSize.x, "Render target width", { "Rendering" }));
	pars.addParam(new Parameter<int>(&gRenderTargetSize.y, "Render target height", { "Rendering" }));
	pars.addParam(new Parameter<float>(&mFps, "FPS", { "Rendering" }, "unsaved"));
	pars.addButton([&]() { mShaderPipeline.saveScreenshot(); }, "Save screenshot", { "Rendering" });
	pars.addParam(new Parameter<bool>(&mEnableRecorder, "Record every frame", { "Rendering" }, "unsaved"));
	pars.addButton([&]() { mDoPerformanceTestOnNextFrame = true; }, "Do GPU bottleneck test", { "Profiler" }, "unsaved");
	pars.addParam(&mDrawFps, "Draw FPS", { "Rendering" });

	// Add sketches here...
	mSketches.push_back(make_shared<Staircase>());

	CHECK_GL;
	checkRenderTargetIsCorrectSize();
	CHECK_GL;
	for (auto& s : mSketches)
	{
		s->setup();
		CHECK_GL_X(s->name)
	}
	gl::clear(Color::black());


	gInput = shared_ptr<Input>(new Input);
	CHECK_GL;

	mShaderPipeline.setup(gRenderTargetSize);
	CHECK_GL;
	g.shaderPipeline->setup(gRenderTargetSize, "GlobalShaderPipeline");
	CHECK_GL;

	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	CHECK_GL;


	pars.setup();

	checkRenderTargetIsCorrectSize();

	gInput->start();
	// For 3D view
	reset3dCam();

	// Setup OSC mappings
	mOsc.addMapping(new LabelMapping("Fade", 300));
	mOsc.addMapping(new FaderMapping("Fade target", 301, &mFadeTarget));
	mOsc.addMapping(new FaderMapping("Fade value", 302, &mFadeValue));
	mOsc.addMapping(new FaderMapping("Fade duration", 303, &mNormalizedFadeDuration));
	mOsc.addMapping(new ToggleMapping("tracking", 14, &g.isTrackingUser));
	mOsc.addMapping(new ToggleMapping("dummy", 17, &mDummyToggle));
	mOsc.addMapping(new FaderMapping("FPS", 18, &mNormalizedFps));
	mOsc.addMapping(new RowMultiToggleMapping("Sketch", 15, &mSelectedSketch));
	
	// Add show-specific OSC mappings here...


	mNonFullScreenWindowPos = getWindow()->getPos();
	mNonFullScreenWindowSize = getWindow()->getSize();

	mOscTransmitter.setup();
}

void ThisFloatingTrackerApp::checkRenderTargetIsCorrectSize()
{
	if (gRenderTargetSize.x <= 0 || gRenderTargetSize.y <= 0)
	{
		gRenderTargetSize = Vec2i(1920, 1080);
	}
	gAspectRatio = float(gRenderTargetSize.x) / gRenderTargetSize.y;
	gRenderArea = Rectf(-gAspectRatio, -1, gAspectRatio, 1);

	if (mShaderPipeline.getSize() != gRenderTargetSize)
	{
		mShaderPipeline.resize(gRenderTargetSize);
	}
	if (g.shaderPipeline->getSize() != gRenderTargetSize)
	{
		g.shaderPipeline->resize(gRenderTargetSize);
	}

}



void ThisFloatingTrackerApp::updateMouseGlobals(MouseEvent event)
{
	g.mouseLeft = event.isLeftDown();
	g.mouseMiddle = event.isMiddleDown();
	g.mouseRight = event.isRightDown();
	g.mouseDeltaWindowCoordinates = event.getPos() - g.mouseWindowCoordinates;
	g.mouseWindowCoordinates = event.getPos();
	float mx = event.getX() / float(getWindowWidth());
	float my = event.getY() / float(getWindowHeight());
	g.dmx = (g.mx - mx) / g.dt;
	g.dmy = (g.my - my) / g.dt;
	g.mx = mx;
	g.my = my;

}

void ThisFloatingTrackerApp::mouseDown(MouseEvent event)
{
	updateMouseGlobals(event);
	cout << "mx " << g.mx << " my " << g.my << "\nnormalized: " << (g.mx * 2 - 1) << "," << (g.my * 2 - 1) << endl;
	if (g.enable3dView)
	{
		g.cam3d.mouseDown(event.getPos());
	}
}

void ThisFloatingTrackerApp::mouseUp(MouseEvent event)
{
	updateMouseGlobals(event);
}

void ThisFloatingTrackerApp::mouseMove(MouseEvent event)
{
	updateMouseGlobals(event);
}

void ThisFloatingTrackerApp::mouseDrag(MouseEvent event)
{
	updateMouseGlobals(event);
	if (g.enable3dView)
	{
		g.cam3d.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
	}
}

void ThisFloatingTrackerApp::mouseWheel(MouseEvent event)
{
	g.mouseWheel = event.getWheelIncrement();
}

void ThisFloatingTrackerApp::keyDown(KeyEvent event)
{
	char key = event.getChar();

	g.keysPressed.insert(event.getCode());
	if (event.getChar() == 'r')
	{
		std::cout << getElapsedSeconds() << ": Recompiling shaders" << std::endl;
		mShaderPipeline.recompileShaders();
		for (auto& ptr : mSketches)
		{
			ptr->recompileShaders();
			CHECK_GL_X(ptr->name)
		}
	}
	else if (event.getChar() == 'm')
	{
		std::cout << "mx " << g.mx << " my " << g.my << endl;
	}
	else if (event.getChar() == 's')
	{
		mShaderPipeline.saveScreenshot();
	}
	else if (event.getChar() == KeyEvent::KEY_BACKSPACE && g.enable3dView)
	{
		reset3dCam();
	}
	else if (event.getChar() == 'f' && event.isControlDown())
	{
		mySetFullScreen(!mIsFullScreen);
	}
	else if (event.getChar() == 'd' && event.isControlDown())
	{
		setDebugDisplay(!mEnableDebugDisplay);
	}
	else if (key=='§' || ('0' <= key && key < '9'))
	{
		if (key == '§')
		{
			key = '0';
		}
			mSelectedSketch = key - '0';
	}
	else if (event.getChar() == 'z')
	{
		mDummyToggle = !mDummyToggle;
	}
	else if (key == 'F')
	{
		mFadeTarget = 1.f;
	}
	else if (key == 'f')
	{
		mFadeTarget = 0.f;
	}

	g.shiftDown = event.isShiftDown();
	g.ctrlDown = event.isControlDown();
	g.altDown = event.isAltDown();
}

void ThisFloatingTrackerApp::setDebugDisplay(bool isDebugDisplay)
{
	mEnableDebugDisplay = isDebugDisplay;
	if (mEnableDebugDisplay)
	{
		showCursor();
	}
	else
	{
		hideCursor();
	}
}

void ThisFloatingTrackerApp::mySetFullScreen(bool isFullScreen)
{
	mIsFullScreen = isFullScreen;

	if (mIsFullScreen)
	{
		mNonFullScreenWindowPos = getWindow()->getPos();
		mNonFullScreenWindowSize = getWindow()->getSize();
		getWindow()->setBorderless(true);
		getWindow()->setAlwaysOnTop(true);
		vector<DisplayRef> displays = Display::getDisplays();
		if (!displays.empty())
		{
			DisplayRef target = nullptr;
			for (DisplayRef const& d : displays)
			{
				if (target == nullptr || d->getSystemCoordinate(Vec2i()).x > target->getSystemCoordinate(Vec2i()).x)
				{
					target = d;
				}
			}
			cout << "Using display at coordinate: " << target->getSystemCoordinate(Vec2i()) << endl;
			getWindow()->setPos(target->getSystemCoordinate(Vec2i()));
			getWindow()->setSize(target->getSize());
		}
	}
	else
	{
		getWindow()->setBorderless(false);
		getWindow()->setAlwaysOnTop(false);
		getWindow()->setPos(mNonFullScreenWindowPos);
		getWindow()->setSize(mNonFullScreenWindowSize);
	}
}

void ThisFloatingTrackerApp::keyUp(KeyEvent event)
{
	g.keysPressed.erase(event.getCode());
	g.shiftDown = event.isShiftDown();
	g.ctrlDown = event.isControlDown();
	g.altDown = event.isAltDown();
}

void ThisFloatingTrackerApp::crossfadeToSketch(int newSketch)
{
	if (newSketch != mSelectedSketch)
	{
		mIsCrossfading = true;
		mFadeTarget = 0.f;
		mCrossfadeTargetSketch = newSketch;
	}
}


void ThisFloatingTrackerApp::update()
{
	assert(gInput != nullptr);

	for (int i = 0; i < mSketches.size(); i++)
	{
		mSketches[i]->isActive = (i == mSelectedSketch);
	}

	float t = getElapsedSeconds();
	g.dt = t - g.time;
	g.dt = max(0.001f, float(g.dt));
	g.time = t;
	mFps = 1.f / g.dt;
	mFpsAverageLast = mFpsAverage.update(mFps);
	mNormalizedFps = mFpsAverageLast / 60.f;

	// update fading touch osc controls
	size_t const N = 1;
	float* const values[N] = { &mFadeValue };
	float const targets[N] = { mFadeTarget };
	float const normalizedDurations[N] = { mNormalizedFadeDuration };
	float const maxDurations[N] = { mMaxFadeDuration };

	for (int i = 0; i < N; i++)
	{
		static_assert(ARRAYSIZE(values) == N && ARRAYSIZE(targets) == N && ARRAYSIZE(normalizedDurations) == N && ARRAYSIZE(maxDurations)==N, "");
		if (*values[i] != targets[i])
		{
			float const duration = normalizedDurations[i] * maxDurations[i];
			float const dx = (targets[i] - *values[i]);
			float const dxdt = sign(dx) * min(abs(dx), abs(g.dt / duration));
			*values[i] += dxdt;
		}
	}
	
	if (mDrawFps)
	{
		mFpsTroughLast = mFpsTrough.update(mFps);
	}


	gInput->update();
	CHECK_GL;
	if (0 <= mSelectedSketch && mSelectedSketch < mSketches.size())
	{
		mSketches.at(mSelectedSketch)->update();
	}
	CHECK_GL;

	mOsc.update();
	mOscTransmitter.update();
	gProfiler.update();
}


#ifdef SM_NVIDIA_PERFORMANCE_KIT
// ********************************************************
// Set up NVPMAPI
#define NVPM_INITGUID
#include "NvPmApi.Manager.h"
// Simple singleton implementation for grabbing the NvPmApi
static NvPmApiManager S_NVPMManager;
NvPmApiManager *GetNvPmApiManager() { return &S_NVPMManager; }
const NvPmApi* getNvPmApi() { return S_NVPMManager.Api(); }
#endif

void ThisFloatingTrackerApp::profiledRender()
{
#ifdef SM_NVIDIA_PERFORMANCE_KIT

#define CHECK_RESULT(call) { NVPMRESULT result = call; if (result != NVPM_OK) { std::cout << "Nvidia Performance Kit returned error "<<result<<" during call \""<<#call<<"\"."<<std::endl; return;}}

	CHECK_RESULT(GetNvPmApiManager()->Construct(L"C:\\Program Files\\PerfKit_4.1.0.14260\\bin\\win7_x64\\NvPmApi.Core.dll"));

	auto api = getNvPmApi();

	CHECK_RESULT(api->Init());

	NVPMContext context;
	CHECK_RESULT(api->CreateContextFromOGLContext((uint64_t)::wglGetCurrentContext(), &context));

	CHECK_RESULT(api->AddCounterByName(context, "GPU Bottleneck"));
	CHECK_RESULT(api->AddCounter(context, 725));

	NVPMUINT nCount(1);
	CHECK_RESULT(api->BeginExperiment(context, &nCount));
	for (NVPMUINT i = 0; i < nCount; i++) {
		CHECK_RESULT(api->BeginPass(context, i));

		CHECK_RESULT(api->BeginObject(context, 0));
		render();
		glFinish();
		CHECK_RESULT(api->EndObject(context, 0));

		CHECK_RESULT(api->EndPass(context, i));
	}
	CHECK_RESULT(api->EndExperiment(context));

	NVPMUINT64 bottleneckUnitId(42424242);
	NVPMUINT64 bottleneckCycles(42424242);
	CHECK_RESULT(api->GetCounterValueByName(context, "GPU Bottleneck", 0, &bottleneckUnitId, &bottleneckCycles));
	char name[1024] = { 0 };
	NVPMUINT length = sizeof(name);
	NVPMRESULT nvResult = api->GetCounterName(bottleneckUnitId, name, &length);
	NVPMUINT64 counterValue(42424242), counterCycles(42424242);
	nvResult = api->GetCounterValue(context, bottleneckUnitId, 0, &counterValue, &counterCycles);

	std::cout << "--- NVIDIA Performance Kit GPU profile ---\n"
		"bottleneckUnitId: " << bottleneckUnitId
		<< ", bottleneckCycles: " << bottleneckCycles
		<< ", unit name: " << name
		<< ", unit value: " << counterValue
		<< ", unit cycles: " << counterCycles
		<< std::endl;

#undef CHECK_RESULT
#else
	std::cout << "NVIDIA Performance Kit was not enabled. Set a macro for SM_NVIDIA_PERFORMANCE_KIT when compiling." << std::endl;
#endif
}

void ThisFloatingTrackerApp::render()
{
	mShaderPipeline.bind();
	CHECK_GL;
	if (0 <= mSelectedSketch && mSelectedSketch < mSketches.size())
	{
		mSketches.at(mSelectedSketch)->draw();
	}
	CHECK_GL;
	mShaderPipeline.unbind();
	CHECK_GL;

	if (g.enable3dView)
	{
		gl::setMatrices(g.cam3d.getCamera());
	}
	else
	{
		// Render to area the correct aspect ratio regardless of window shape
		setMatricesRender(true);
	}

	gl::Fbo shaderPipelineOutput = mShaderPipeline.render();
	CHECK_GL;
	if (mEnableDebugDisplay)
	{
		gl::clear(ColorA(0.15, 0.15, 0.15, 1.));
	}
	else
	{
		gl::clear(Color::black());
	}
	gl::color(Color(1,0,0));
	glDisable(GL_TEXTURE_2D);
	float alpha = ci::easeInOutQuad(mFadeValue);
	gl::color(ColorA(alpha, alpha, alpha, 1.f));
	gl::enableAlphaBlending();
	gl::draw(shaderPipelineOutput.getTexture(0), gRenderArea);
	gl::disableAlphaBlending();
	CHECK_GL;
}

void ThisFloatingTrackerApp::draw()
{
	assert(gInput != nullptr);
	gl::setViewport(getWindowBounds());
	if (mEnableDebugDisplay)
	{
		gl::clear(Color(0., 0.2, 0.));
	}
	else
	{
		gl::clear(Color::black());
	}

	if (mDoPerformanceTestOnNextFrame)
	{
		profiledRender();
		mDoPerformanceTestOnNextFrame = false;
		return;
	}
	else
	{
		render();
	}

	if (mEnableDebugDisplay)
	{
		gInput->draw();
		CHECK_GL;
		pars.draw();
		CHECK_GL;
		if (mDrawFps)
		{
			stringstream ss;
			ss << "FPS: " << setprecision(0) << fixed << mFpsAverageLast << " Trough: " << mFpsTroughLast;
			gl::color(ColorA::white());
			drawStringRenderSpace(ss.str(), Vec2f(gRenderArea.x1, gRenderArea.y1));
		}
		CHECK_GL;
	}
	if (mEnableRecorder)
	{
		static fs::path basePath = "screenshot_" + getDateString() + "_";
		ci::Surface8u screenshot(getWindowWidth(), getWindowHeight(), false, SurfaceChannelOrder::RGB);
		glReadPixels(0, 0, getWindowWidth(), getWindowHeight(), GL_RGB, GL_UNSIGNED_BYTE, screenshot.getData());
		cv::Mat tmp = toOcv(screenshot).clone();
		cv::flip(tmp, tmp, 0);
		screenshot = fromOcv(tmp);
		fs::path path = basePath;
		path += to_string(getElapsedFrames()) + ".png";
		ci::writeImage(path, screenshot);
	}
}

void ThisFloatingTrackerApp::resize()
{
	g.windowSize = getWindowSize();
}


CINDER_APP_NATIVE(ThisFloatingTrackerApp, RendererGl)

int main(int argc, char* argv[])
{
	cinder::app::AppBasic::prepareLaunch();
	cinder::app::AppBasic *app = new ThisFloatingTrackerApp;
	cinder::app::RendererRef ren(new RendererGl);
	cinder::app::AppBasic::executeLaunch(app, ren, "This Floating Tracker");
	cinder::app::AppBasic::cleanupLaunch();
	return 0;
}




