#pragma once

#include "Sketch.h"
#include "Shader.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"



class Staircase : public Sketch
{
public:
	Staircase();
	virtual void setup() override;
	virtual void update() override;
	virtual void draw() override;
	virtual void recompileShaders() override;

	int mNumVerticesX, mNumVerticesY;

private:
	void setupVbo();


	ShaderPtr mShader;
	ci::gl::VboMeshRef mVbo;
};