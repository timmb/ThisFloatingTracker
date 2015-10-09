//
//  ShaderPipeline.h
//  SlowMovementSketches
//
//  Created by Tim Murray-Browne on 06/04/2014.
//
//

#pragma once
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Filesystem.h"
#include "Params.h"
#include "Shader.h"


class ShaderPipeline
{
public:
	ShaderPipeline(bool retainOriginalImage=false);
	virtual ~ShaderPipeline();
	
	void setShaders(std::vector<ShaderPtr> shaders) { mShaders = shaders; }
	void recompileShaders();
	void setup(ci::Vec2i const& renderSize, std::string const& moduleName="ShaderPipeline");
	void resize(ci::Vec2i newSize);
	ci::Vec2i getSize() const;
	void bind();
	void unbind();
	/// \return An fbo object containing the most recent shader pass, ready to be drawn to the final render target.
	ci::gl::Fbo render();
	/// \return An fbo object before and shaders are applied. Still works after render() is called but only
	/// if retainOriginalImage was true in constructor of this object. Otherwise an empty Fbo object is
	/// returned.
	ci::gl::Fbo getOriginalImage() const { assert(mRetainOriginalImage); return mFbos[2]; }
	void saveScreenshot() { mSaveScreenshotNextDraw = true; }

private:
	/// Keep the original image before shader passes saved on its own FBO
	bool mRetainOriginalImage;

	// Use 3 fbos: if mRetainOriginalImage then 2 is used for client rendering. Otherwise 0 is used
	ci::gl::Fbo mFbos[3];
	std::vector<ShaderPtr> mShaders;
	std::unique_ptr<ci::gl::SaveFramebufferBinding> mSaveFramebufferBinding;
	bool mSaveScreenshotNextDraw;
	/// When bound we save the original gRenderTarget in here. May be Fbo()
	ci::Area mOriginalViewport;
	ci::gl::Fbo mOriginalRenderTarget;
};


