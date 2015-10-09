//
//  ShaderPipeline.cpp
//  SlowMovementSketches
//
//  Created by Tim Murray-Browne on 06/04/2014.
//
//

#include "ShaderPipeline.h"
#include "Common.h"

#include "cinder/app/App.h"


using namespace ci;
using namespace std;
using namespace ci::gl;


ShaderPipeline::ShaderPipeline(bool retainOriginalImage)
: mSaveScreenshotNextDraw(false)
, mRetainOriginalImage(retainOriginalImage)
{
}


ShaderPipeline::~ShaderPipeline()
{
	
}

void ShaderPipeline::setup(Vec2i const& renderSize, std::string const& moduleName)
{
	resize(renderSize);
	recompileShaders();
}

void ShaderPipeline::recompileShaders()
{
	for (ShaderPtr shader: mShaders)
	{
		shader->compile();
	}
}


Vec2i ShaderPipeline::getSize() const
{
	if (!mFbos[0])
	{
		return Vec2i();
	}
	return mFbos[0].getSize();
}


void ShaderPipeline::resize(Vec2i size)
{
	if (size == Vec2i())
	{
		return;
	}
	Fbo::Format format;
	format.setMagFilter(GL_LINEAR);
	format.setMinFilter(GL_LINEAR);
	//format.setSamples(16);
	//	format.setCoverageSamples(4);
	for (int i=0; i<3; i++)
	{
		if (i == 2 && !mRetainOriginalImage)
		{
			break;
		}
		/// fbo passed to client should be multisampled
		if ((i == 0 && !mRetainOriginalImage) || (i == 2 && mRetainOriginalImage))
		{
			format.setSamples(16);
		}
		else
		{
			format.setSamples(0);
		}
		mFbos[i] = Fbo(size.x, size.y, format);
		mFbos[i].bindFramebuffer();
		gl::clear(ColorA(0.f, 0.f, 0.f, 0.f));
		mFbos[i].unbindFramebuffer();
	}
	//if (keepOriginalImage)
	//{
	//	format.setSamples(0);
	//	mOriginalImageFbo = Fbo(size.x, size.y, format);
	//	mOriginalImageFbo.bindFramebuffer();
	//	gl::clear(ColorA(0.f, 0.f, 0.f, 1.f));
	//	mOriginalImageFbo.unbindFramebuffer();
	//}
}

void ShaderPipeline::bind()
{
	mOriginalViewport = gl::getViewport();
	mSaveFramebufferBinding = unique_ptr<SaveFramebufferBinding>(new SaveFramebufferBinding);
	mOriginalRenderTarget = gRenderTarget;
	int fboIndex = mRetainOriginalImage ? 2 : 0;
	gRenderTarget = mFbos[fboIndex];
	mFbos[fboIndex].bindFramebuffer();
	gl::setViewport(mFbos[fboIndex].getBounds());
	gl::clear(ColorA(0., 0., 0., 0.));
}

void ShaderPipeline::unbind()
{
	int fboIndex = mRetainOriginalImage ? 2 : 0;
	mFbos[fboIndex].unbindFramebuffer();
	mSaveFramebufferBinding = nullptr;
	gRenderTarget = mOriginalRenderTarget;
	if (gRenderTarget != gl::Fbo())
	{
		gRenderTarget.bindFramebuffer();
	}
	mOriginalRenderTarget = gl::Fbo();
	CHECK_GL;
	gl::setViewport(mOriginalViewport);
	mOriginalViewport = Area();
	CHECK_GL;
}

ci::gl::Fbo ShaderPipeline::render()
{
	glEnable(GL_TEXTURE_2D);
	gl::color(Color::white());
	gl::disableAlphaBlending();
	int source = mRetainOriginalImage? 2 : 0;
	gl::pushMatrices();
	assert(abs(gAspectRatio - mFbos[source].getAspectRatio()) < 0.001);
	ci::Area origViewport = gl::getViewport();
	gl::setViewport(mFbos[source].getBounds());
	gl::setMatricesWindow(mFbos[source].getSize(), false);
	for (ShaderPtr shader: mShaders)
	{
		if (!shader->isEnabled() || !shader->isLoaded())
		{
			continue;
		}
		// start with either 0 or 3 then alternate between 0 and 1
		int target = source==0? 1 : source==1? 0 : source==2? 0 : 42;
		//int target = 1 - source;
		assert(target < 3);
		mFbos[source].bindTexture();
		mFbos[target].bindFramebuffer();
		shader->bind();
		shader->program->uniform("tex", 0);
		gl::drawSolidRect(mFbos[target].getBounds());
		shader->unbind();
		mFbos[target].unbindFramebuffer();
		mFbos[source].unbindTexture();
		source = source ==0? 1 : source==1? 0 : source==2? 0 : 42;
		assert(source < 3);
		//source = 1 - source;
	}
	if (mSaveScreenshotNextDraw)
	{
		Vec2i size = mFbos[source].getSize();
		if (isPowerOf2(size.x) && isPowerOf2(size.y))
		{
			mFbos[source].bindTexture();
			vector<unsigned char> data(size.x * size.y * 3);
			glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data.data());
			mFbos[source].unbindTexture();
			Surface image(data.data(), size.x, size.y, size.x * 3, SurfaceChannelOrder(SurfaceChannelOrder::RGB));
			writeImage(fs::path("screenshot_" + getDateString() + ".png"), image);
		}
		else
		{
			cerr << "Error: Cannot save screenshot as render target dimensions are not powers of two." << endl;
		}
		mSaveScreenshotNextDraw = false;
	}

	gl::popMatrices();
	gl::setViewport(origViewport);
	if (gRenderTarget != gl::Fbo())
	{
		gRenderTarget.bindFramebuffer();
	}
	return mFbos[source];
}





