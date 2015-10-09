#pragma once
#include "cinder/gl/GlslProg.h"
#include <string>
#include "cinder/Filesystem.h"
#include <memory>

class BaseParameter;

class Shader
{
public:
	Shader(std::string const& name_, 
		std::string const& module_,
		ci::fs::path const& fragmentPath, 
		ci::fs::path const& vertexPath="vert_passThrough.glsl",
		ci::fs::path const& geometryPath="");

	Shader(Shader const& rhs) = delete;
	Shader& operator=(Shader const& rhs) = delete;
	virtual ~Shader();

	ci::gl::GlslProgRef program;
	std::string module;
	std::string name;
	
	void bind();
	void unbind();
	
	void setEnabled(bool isEnabled) { mIsEnabled = isEnabled; }
	bool isEnabled() const { return mIsEnabled; }
	
	bool compile();
	bool isLoaded() const { return mIsLoaded; }
	
protected:
	void printCompileError(std::string const& message) const;
	virtual void setUniforms() {}
	
	ci::fs::path mVertexPath;
	ci::fs::path mFragmentPath;
	ci::fs::path mGeometryPath;
	bool mIsEnabled;
	bool mIsLoaded;
	
	std::vector<std::shared_ptr<BaseParameter>> mParams;
};

typedef std::shared_ptr<Shader> ShaderPtr;

class ShaderBlurPass : public Shader
{
public:
	ShaderBlurPass(std::string const& name_, std::string const& module_, bool isHorizontal);

	/// Saved parameter controlling amount of blur
	float amount;
	/// amount above is multiplied by this unsaved parameter, allowing frame-by-frame control
	float scaleAmount;
	/// After blur, values are multiplied by this
	float scaleOutput;

	virtual void setUniforms() override
	{
		program->uniform("amount", amount * scaleAmount);
		program->uniform("isHorizontal", mIsHorizontal);
		program->uniform("scaleOutput", scaleOutput);
	}
	
protected:
	bool mIsHorizontal;
};

class ShaderHorizontalBlur : public ShaderBlurPass
{
public:
	ShaderHorizontalBlur(std::string const& module, std::string const& appendToName = "")
	: ShaderBlurPass("horizontal blur"+appendToName, module, true)
	{}
};

class ShaderVerticalBlur : public ShaderBlurPass
{
public:
	ShaderVerticalBlur(std::string const& module, std::string const& appendToName="")
		: ShaderBlurPass("vertical blur" + appendToName, module, false)
	{}
};
	
class ShaderBloom : public Shader
{
public:
	ShaderBloom(std::string const& module_)
	: Shader("Bloom", module_, "frag_bloom.glsl")
	, amount(0)
	{}

	float amount;

	virtual void setUniforms() override
	{
		program->uniform("amount", amount);
	}
};





