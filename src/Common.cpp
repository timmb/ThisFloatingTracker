#include "Common.h"
#include "Cinder/App/AppNative.h"
#include "cinder/Rect.h"
#include "cinder/Area.h"
#include "cinder/Font.h"
#include "Globals.h"

using namespace ci;

Params pars;
std::shared_ptr<Input> gInput;

float width(float proportion)
{
	return proportion * 2 - 1.f;
}

float height(float proportion)
{
	return proportion * 2 - 1.f;
}


void setMatricesIdentity()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

int printGlError(char const* file, int line, char const* info)
{
	GLenum glErr;

	glErr = glGetError();
	if (glErr != GL_NO_ERROR)
	{
		char const* error = "";
		switch (glErr)
		{
		case GL_INVALID_ENUM: error = "GL_INVALID_ENUM"; break;
		case GL_INVALID_VALUE: error = "GL_INVALID_VALUE"; break;
		case GL_INVALID_OPERATION: error = "GL_INVALID_OPERATION"; break;
		case GL_INVALID_FRAMEBUFFER_OPERATION: error = "GL_INVALID_FRAMEBUFFER_OPERATION"; break;
		case GL_OUT_OF_MEMORY: error = "GL_OUT_OF_MEMORY"; break;
		default: error = "unknown error value"; break;
		}
		printf("glError in file %s @ line %d (%s): %#010x: %s\n",
			file, line, info, glErr, error);
	}
	return glErr;
}

bool isBeyondBleed(ci::Vec2f const& v)
{
	const float bleedZone = 0.3f;
	const Rectf insideBleed(Vec2f(-1, -1)*(1.f + bleedZone), Vec2f(1, 1)*(1.f + bleedZone));
	return !insideBleed.contains(v);
}

Rectf gRenderArea(-1, -1, 1, 1);

float getRenderWidth()
{
	return gRenderArea.getWidth();
}

float getRenderHeight()
{
	return gRenderArea.getHeight();
}

namespace
{
	/// Get the rectf of the area of windowbounds that actually
	/// is being rendered to by setMatricesRender, which will avoid
	/// aspect ratio distortion.
	Rectf getAspectCorrectedRenderTargetArea()
	{
		Rectf renderTargetArea(Vec2f(), g.windowSize);
		float windowAspect = g.windowSize.x / (float)g.windowSize.y;
		// render area is limited to maintain aspect ratio
		if (windowAspect > gAspectRatio)
		{
			// window wider than render target, reduce width
			renderTargetArea.x2 = renderTargetArea.getHeight() * gAspectRatio;
		}
		else if (windowAspect < gAspectRatio)
		{
			// window taller than render target, reduce height
			renderTargetArea.y2 = renderTargetArea.getWidth() / gAspectRatio;
		}
		return renderTargetArea;
	}
}

Vec2f mapWindowToRender(Vec2f point)
{
	Rectf renderTargetArea = getAspectCorrectedRenderTargetArea();
	point.y = renderTargetArea.getHeight() - point.y;
	return point / renderTargetArea.getSize() * Vec2f(gRenderArea.getSize()) + Vec2f(gRenderArea.x1, gRenderArea.y1);
}

Vec2f mapWindowDeltaToRender(Vec2f point)
{
	Rectf renderTargetArea = getAspectCorrectedRenderTargetArea();
	point.y = - point.y;
	return point / renderTargetArea.getSize() * Vec2f(gRenderArea.getSize());
}

void setMatricesRender(bool finalRender, Rectf const& renderArea)
{
	if (finalRender || gRenderTarget == nullptr)
	{
		// Set the matrices so that gRenderArea is completely contained within the window. If the aspect of the window
		// differs then we leave that below/right of the render area.
		float aspect = renderArea.getAspectRatio() / (g.windowSize.x / (float)g.windowSize.y);

		gl::setMatricesWindow(g.windowSize, false);
		//ci::Matrix44f skew = ci::Matrix44f::identity();
		//// increase y with positive x values by keystone amount. now extreme x values may be up to `keystone` above
		//// the limit for y so scale down too.
		//skew.at(1, 0) = g.horizontalKeystone;
		//skew.scale(Vec3f(1, 1 / (1 + g.horizontalKeystone), 1));
		//gl::multProjection(skew);

		float scale = 1.f;
		if (aspect >= 1.f)
		{
			// render shape is wider than window. stretch based on width
			scale /= renderArea.getWidth() / g.windowSize.x;
		}
		else if (aspect < 1.f)
		{
			scale /= renderArea.getHeight() / g.windowSize.y;
		}
		// origin is now in lower left of screen. if aspect is >1 then shift it up to make
		// the render area touch the top of the window (shift it to lower left of render area)
		if (aspect > 1)
		{
			gl::translate(0, g.windowSize.y - renderArea.getHeight() * scale);
		}
		gl::scale(scale, scale);
		// origin will now be in lower left of render area. translate to move it to centre of render area
		// (cinders Area methods are upside down as we're using y going upwards but it uses y going downwards)
		gl::translate(-renderArea.getUpperLeft());
	}
	else
	{
		gl::setMatricesWindow(gRenderTarget.getSize(), false);
		gl::scale(Vec2f(gRenderTarget.getSize()) / renderArea.getSize());
		gl::translate(-renderArea.getUpperLeft());
	}

}

void reset3dCam()
{
	//float fovDegrees = 70.f;

	float eyeX = gRenderArea.getWidth() / 2.0f;
	float eyeY = gRenderArea.getHeight() / 2.0f;
	//float halfFov = 3.14159f * fovDegrees / 360.0f;
	//float theTan = cinder::math<float>::tan(halfFov);
	//float dist = eyeY / theTan;
	float aspect = gRenderArea.getAspectRatio();

	float theTan = eyeY / gInput->getProjectionWallDistance();
	float halfFovRadians = cinder::math<float>::atan(theTan);
	float fovDegrees = 360.f / 3.14159f * halfFovRadians;

	float nearPlane = 0.1f;
	float farPlane = 40.f;

	fovDegrees = clamp(fovDegrees, 45.f, 110.f);

	CameraPersp cam;
	cam.setPerspective(fovDegrees, aspect, nearPlane, farPlane);
	cam.lookAt(Vec3f(0, 0, 0), Vec3f(0, 0, -gInput->getProjectionWallDistance()));
	//cam.lookAt(Vec3f(0, 0, dist), Vec3f(0, 0, 0.0f));
	g.cam3d.setCurrentCam(cam);
}

vector<ColorA> const gDebugColors = {
	ci::ColorA(0.10196078431372549, 0.7372549019607844, 0.611764705882353, 1.0),
	ci::ColorA(0.9450980392156862, 0.7686274509803922, 0.058823529411764705, 1.0),
	ci::ColorA(0.1803921568627451, 0.8, 0.44313725490196076, 1.0),
	ci::ColorA(0.9019607843137255, 0.49411764705882355, 0.13333333333333333, 1.0),
	ci::ColorA(0.20392156862745098, 0.596078431372549, 0.8588235294117647, 1.0),
	ci::ColorA(0.9058823529411765, 0.2980392156862745, 0.23529411764705882, 1.0),
	ci::ColorA(0.6078431372549019, 0.34901960784313724, 0.7137254901960784, 1.0),
	ci::ColorA(0.08627450980392157, 0.6274509803921569, 0.5215686274509804, 1.0),
	ci::ColorA(0.9529411764705882, 0.611764705882353, 0.07058823529411765, 1.0),
	ci::ColorA(0.1607843137254902, 0.5019607843137255, 0.7254901960784313, 1.0),
	ci::ColorA(0.7529411764705882, 0.2235294117647059, 0.16862745098039217, 1.0),
	ci::ColorA(0.1803921568627451, 0.8, 0.44313725490196076, 1.0),
	ci::ColorA(0.9019607843137255, 0.49411764705882355, 0.13333333333333333, 1.0),
	ci::ColorA(0.8274509803921568, 0.32941176470588235, 0.0, 1.0),
	ci::ColorA(0.6078431372549019, 0.34901960784313724, 0.7137254901960784, 1.0)
};

ci::gl::TextureFontRef gDebugFont;
ci::gl::TextureFontRef gDebugFontMedium;
ci::gl::TextureFontRef gDebugFontBig;
void drawStringRenderSpace(std::string const& s, ci::Vec2f const& pos)
{
	// to [0,1]x[0,1]
	Vec2f posNormalizedSpace = (pos - gRenderArea.getUpperLeft()) / gRenderArea.getSize();
	Vec2f posTargetSpace = posNormalizedSpace * Vec2f(gRenderTargetSize);
	gl::pushMatrices();
	setMatricesRender(true, Rectf(Vec2f(), gRenderTargetSize));
	//gl::setMatricesWindow(gRenderTargetSize, false);
	gl::translate(posTargetSpace);
	gl::scale(1, -1);
	gDebugFontBig->drawString(s, Vec2f());
	gl::popMatrices();
}


void drawStrokedTriangle(Vec2f const& p, float r)
{
	float const angle = float(M_PI * 2 / 3);
	gl::drawStrokedTriangle(p + r * Vec2f(cos(0 * angle), sin(0 * angle)),
		p + r * Vec2f(cos(1 * angle), sin(1 * angle)),
		p + r * Vec2f(cos(2 * angle), sin(2 * angle)));
}


ci::gl::Fbo gRenderTarget;
ci::Vec2i gRenderTargetSize(1920, 1080);
float gAspectRatio = 1.0;

std::string getDateString()
{
	// get date string
	time_t t = time(0);   // get time now
	struct tm now;
	localtime_s(&now, &t);
	char buf[128];
	strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &now);
	return string(buf);
}