#pragma once
#include <string>
#include <boost/circular_buffer.hpp>
#include "Params.h"

class Grapher
{
public:
	/// \param group, e.g. Input
	/// \param excludeFromParams - don't add isEnabled toggle to params
	Grapher(std::string const& name, std::string const& group, bool excludeFromParams=false, int instanceId=-1);
	virtual ~Grapher();

	void update(float newValue);
	void draw();

	std::string name;
	bool isEnabled;
	/// used for colouring
	int instanceId;
	boost::circular_buffer<float> buffer;

private:
	static int sNextInstanceId;
	ParameterPtrT<bool> mIsEnabledParam;
};