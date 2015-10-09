#pragma once
#include "Params.h"
#include "cinder/gl/gl.h"
#include "Kinect.h"
#include "input/Input.h"
#include "cinder/gl/TextureFont.h"
#include "cinder/gl/Fbo.h"
#include <stdint.h>
#include <unordered_set>
#include "utils/Profiler.h"

//namespace tmb
//{

enum
{
	SCENE_SHOW_END = 4
};

float width(float proportion);
float height(float proportion);

/// \return true if v is off screen and beyond the bleed zone
/// assuming normalized coordinate space
bool isBeyondBleed(ci::Vec2f const& v);
inline bool isBeyondBleed(ci::Vec3f const& v) { return isBeyondBleed(v.xy()); }

#if defined DEBUG || defined _DEBUG
	#define CHECK_GL_X(info) printGlError(__FILE__, __LINE__, info);
#define CHECK_GL CHECK_GL_X("")
#else
	#define CHECK_GL
	#define CHECK_GL_X(info)
#endif
int printGlError(char const* file, int line, char const* info);

static int const NUM_JOINTS = JointType_Count;



/// The renderArea, e.g. [-1.4,-1] - [1.4,1] 
extern ci::Rectf gRenderArea;
extern float gAspectRatio;
float getRenderWidth();
float getRenderHeight();

/// Map pixel coordinates to render area including offset
ci::Vec2f mapWindowToRender(Vec2f point);
/// Map pixel coordinates to render area not including offset 
ci::Vec2f mapWindowDeltaToRender(Vec2f point);
inline ci::Vec2f mapWindowToRender(Vec2i const& point) { return mapWindowToRender(Vec2f(point)); }


// Target for final rendering. This is modified by the shader pipeline for
// sketches to rebind if they need to bind their own fbo
extern ci::gl::Fbo gRenderTarget;
extern ci::Vec2i gRenderTargetSize;
//ci::Vec2i getRenderTargetSize();


void setMatricesIdentity();
/// Set the matrices to have the render area cover
/// the framebuffer.
/// Set finalRender to true if using this to render to the screen
/// buffer and aspect ratio considerations will happen. Usually, only
/// the shader pipeline will have this set to true.
/// if gRenderTarget is null then this will always work as if on
/// a final render
/// The resulting render space will cause gRenderArea to fill the whole render target.
/// normally this is [-r, -1] - [r, +1] but can be something else (e.g. for drawing text)
/// y points upwards in renderArea, so y1 is lower, y2 is upper
void setMatricesRender(bool finalRender = false, ci::Rectf const& renderArea = gRenderArea);


extern Params pars;
extern std::shared_ptr<Input> gInput;
extern std::vector<ci::ColorA> const gDebugColors;
extern ci::gl::TextureFontRef gDebugFont;
extern ci::gl::TextureFontRef gDebugFontMedium;
extern ci::gl::TextureFontRef gDebugFontBig;
/// draw a string given render coordinates
void drawStringRenderSpace(std::string const& s, Vec2f const& pos);
/// draw an equilateral triangle
void drawStrokedTriangle(Vec2f const& pos, float radius);

void reset3dCam();

inline
ci::ColorA getDebugColor(int i, float alpha=1.f)
{
	ColorA c = gDebugColors[i%gDebugColors.size()];
	c.a = alpha;
	return c;
}


inline float fastInvSqrt(float x) {
	float xhalf = 0.5f * x;
	int i = *(int*)&x;         // evil floating point bit level hacking
	i = 0x5f3759df - (i >> 1);  // what the fuck?
	x = *(float*)&i;
	x = x*(1.5f - (xhalf*x*x));
	return x;
}

template <typename T>
T sign(T x)
{
	return x < 0 ? -T(1) : T(1);
}

template <typename T>
T clamp(T x, T min = 0, T max = 1)
{
	return x<min ? min : x>max ? max : x;
}

template <typename T>
T mod(T x, T n)
{
	return x < 0 ? x%n + n : x%n;
}

/// http://stackoverflow.com/a/14675998/794283
/// Fast non-euclidean angle on manhattan metric. Suitable for comparisons.

template <typename T>
T diamondAngle(T y, T x)
{
	if (y >= 0)
		return (x >= 0 ? y / (x + y) : 1 - x / (-x + y));
	else
		return (x < 0 ? 2 - y / (-x - y) : 3 + x / (x - y));
}

inline
float diamondAngle(Vec2f const& v)
{
	return diamondAngle(v.y, v.x);
}

/// Think this might be dodgey.
inline
float fastAngle(Vec2f const& v)
{
	static Vec2f const xAxis = Vec2f::xAxis();
	return acos(v.dot(xAxis)*fastInvSqrt(v.lengthSquared()));
}

std::string getDateString();

inline ci::Vec3f fromOcv(cv::Vec3f const& v)
{
	return ci::Vec3f(v[0], v[1], v[2]);
}

template <typename T, typename Scalar=float>
inline T mix(T const& lhs, T const& rhs, Scalar amount = Scalar(0.5))
{
	return amount * lhs + (Scalar(1) - amount) * rhs;
}


/// \return true if matrix is big enough to contain region
inline
bool isBigEnough(cv::Mat const& mat, cv::Rect const& region)
{
	return region.x >= 0 
		&& region.y >= 0 
		&& region.x + region.width <= mat.cols 
		&& region.y + region.height <= mat.rows;
}

template <typename T>
inline
T sq(T const& v)
{
	return v * v;
}

/// map [lowerBound,upperBound] -> [0,1]
template <typename T>
inline
T normalize(T const& value, T const& lowerBound, T const& upperBound)
{
	return (value - lowerBound) / (upperBound - lowerBound);
}

/// Test if element is in collection
template <typename Collection, typename T>
bool isElem(Collection const& c, T const& t)
{
	return std::find(begin(c), end(c), t) != end(c);
}

/// std::find over collection
template <typename Collection, typename T>
auto find(Collection const& c, T const& t) -> decltype(std::begin(c))
{
	return std::find(std::begin(c), std::end(c), t);
}

/// std::find_if over collection
template <typename Collection, typename Pred>
auto find_if(Collection const& c, Pred p) -> decltype(std::begin(c))
{
	return std::find_if(std::begin(c), std::end(c), p);
}


namespace std
{
	template <typename T>
	std::ostream& operator<<(std::ostream& out, std::vector<T> const& rhs)
	{
		out << '[';
		auto it = begin(rhs);
		if (it != end(rhs))
		{
			out << *(it++);
		}
		while (it != end(rhs))
		{
			out << ", " << *(it++);
		}
		return out << ']';
	}

}
