#include "Grapher.h"
#include "cinder/gl/gl.h"
#include "Common.h"
#include <boost/lexical_cast.hpp>
#include <sstream>

using namespace ci;
int Grapher::sNextInstanceId = 0;

Grapher::Grapher(std::string const& name_, std::string const& group, bool excludeFromParams, int instanceId)
: isEnabled(false)
, instanceId(instanceId<0? sNextInstanceId++ : instanceId)
, name(name_)
, buffer(100)
{
	if (!excludeFromParams)
	{
		mIsEnabledParam = pars.addParam(&isEnabled, "Graph_" + name, { group });
	}
}

Grapher::~Grapher()
{
	if (mIsEnabledParam != nullptr)
	{
		pars.removeParam(mIsEnabledParam);
	}
}


void Grapher::update(float newValue)
{
	if (isEnabled)
	{
		buffer.push_back(newValue);
	}
}


void Grapher::draw()
{
	if (!isEnabled || buffer.empty())
	{
		return;
	}

	vector<Vec2f> data;
	data.reserve(buffer.size());
	{
		auto it(begin(buffer)), it_end(end(buffer));
		float x = gRenderArea.x1;
		float step = gRenderArea.getWidth() / buffer.capacity();
		for (; it != it_end; ++it, x += step)
		{
			data.push_back(Vec2f(x, *it * 2 - 1));
		}
	}

	gl::pushMatrices();
	setMatricesRender();
	gl::translate(gRenderArea.getWidth() * (buffer.capacity() - buffer.size()) / float(buffer.capacity()), 0);
	glDisable(GL_TEXTURE_2D);
	glLineWidth(0.3f);
	gl::color(getDebugColor(instanceId));
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, 0, data.data());
	glDrawArrays(GL_LINE_STRIP, 0, data.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	gl::popMatrices();
}