#include "Shader.h"
#include "cinder/DataSource.h"
#include "Common.h"
#include "cinder/app/App.h"
#include "Globals.h"

using namespace std;
using namespace ci;

Shader::Shader(string const& name_, string const& module_, ci::fs::path const& fragmentPath, fs::path const& vertexPath, fs::path const& geometryPath)
: name(name_)
, module(module_)
, mVertexPath(vertexPath)
, mFragmentPath(fragmentPath)
, mGeometryPath(geometryPath)
, mIsEnabled(true)
, mIsLoaded(false)
{
	compile();
	mParams.push_back(pars.addParam(new Parameter<bool>(&mIsEnabled, "Enable " + name + " shader", { module })));
}

Shader::~Shader()
{
}

bool Shader::compile()
{
	mIsLoaded = false;
	DataSourceRef shaderDataFrag, shaderDataVert, shaderDataGeom;
	try
	{
		shaderDataFrag = app::loadAsset(mFragmentPath);
	}
	catch (app::AssetLoadExc const& e)
	{
		app::console() << "ERROR: Failed to open asset " << mFragmentPath << endl;
		return false;
	}
	try
	{
		shaderDataVert = app::loadAsset(mVertexPath);
	}
	catch (app::AssetLoadExc const& e)
	{
		app::console() << "ERROR: Failed to open asset " << mVertexPath << endl;
		return false;
	}
	if (mGeometryPath != "")
	{
		try
		{
			shaderDataGeom = app::loadAsset(mGeometryPath);
		}
		catch (app::AssetLoadExc const& e)
		{
			app::console() << "ERROR: Failed to open asset " << mGeometryPath << endl;
			return false;
		}
	}
	try
	{
		CHECK_GL_X(name.c_str());
		program = gl::GlslProg::create(shaderDataVert, shaderDataFrag, shaderDataGeom);
		CHECK_GL_X(name.c_str());
		GLint isLinked(0);
		glGetProgramiv(program->getHandle(), GL_LINK_STATUS, &isLinked);
		if (isLinked == GL_FALSE)
		{
			char logBuffer[4096];
			GLsizei len(0);
			glGetProgramInfoLog(program->getHandle(), sizeof(logBuffer), &len, logBuffer);
			printCompileError("Link error: "+string(logBuffer, logBuffer+len));
			return false;
		}
		mIsLoaded = true;
		return true;
	}
	catch (gl::GlslProgCompileExc const& e)
	{
		printCompileError(e.what());
		return false;
	}
}

void Shader::printCompileError(std::string const& message) const
{
	app::console() << "\n*~*~*~*~*~*~*~*~*~*~*~*~*~*~*"
		"\nERROR when compiling shader " << name << ":\n" << message << endl;

}


void Shader::bind()
{
	if (isLoaded())
	{
		program->bind();
		program->uniform("mx", g.mx);
		program->uniform("my", g.my);
		program->uniform("qom", 0.f);
		//program->uniform("cumulativeQom", g.cumulativeQom);
		//program->uniform("userPosFiltered", g.userPosFiltered);
		//program->uniform("userVel", g.userVel);
		program->uniform("time", (float) app::getElapsedSeconds());
		// get mvp matrix.
		ci::Matrix44f pvm = gl::getProjection() * gl::getModelView();
		program->uniform("pvm", pvm);
		setUniforms();
	}
}

void Shader::unbind()
{
	if (isLoaded())
	{
		program->unbind();
	}
}



ShaderBlurPass::ShaderBlurPass(std::string const& name_, std::string const& module_, bool isHorizontal)
	: Shader(name_, module_, "frag_blur.glsl")
	, mIsHorizontal(isHorizontal)
	, amount(0)
	, scaleAmount(1.f)
	, scaleOutput(1.f)
{
	pars.addParam(new Parameter<float>(&amount, name + " amount", { module_ }));
	pars.addParam(new Parameter<float>(&scaleOutput, name + " scale output", { module_ }));
}