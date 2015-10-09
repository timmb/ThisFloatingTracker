//
//  Params.cpp
//
//  Created by Tim Murray-Browne on 2014-04-06.
//
//

#include "Params.h"
#include "Common.h"
#include "cinder/app/AppBasic.h"
#include "json/json.h"
#include <fstream>
#include <iostream>
#include <ctime>

#include "../src/AntTweakBar/AntTweakBar.h"

using namespace std;
using namespace ci;


Params::Params()
	: mHasSetupBeenCalled(false)
{
}

Params::~Params()
{
	clear();
}

void Params::clear()
{
	Lock lock(mMutex);
	mParams = params::InterfaceGl();
	mParameters.clear();
	mRoot = Json::Value();
	mHasSetupBeenCalled = false;
}

bool Params::load(string const& filename)
{
	Lock lock(mMutex);

	mJsonFile = filename;
	
	
	ifstream in(filename.c_str(), ifstream::in);
	bool success = true;
	try {
		in >> mRoot;
	}
	catch (std::runtime_error& e)
	{
		cerr << ("Error parsing json file "+filename, "Settings") << endl;
		success = false;
	}
	if (success)
	{
		//app::console() << "successful parse" << endl;
		cout << "Successfully loaded "+filename << endl;
	}
	if (!success)
	{
		cerr << "Problem with settings json file." << endl;
	}
	return success;
}

void Params::save()
{
	save(mJsonFile);
}


void Params::snapshot()
{
	// get date string
	time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
	char buf[128];
	strftime(buf, sizeof(buf), "-%Y-%m-%d-%H-%M-%S", now);

	int extensionPos = mJsonFile.find(".json");
	string filestem = mJsonFile.substr(0, extensionPos);
	
	save(filestem + buf + ".json");
}

void Params::save(string const& filename)
{
	Lock lock(mMutex);

	Json::Value root;
	for (auto it=mParameters.begin(); it!=mParameters.end(); ++it)
	{
		if ((**it).isSaved)
		{
			(**it).writeJson(root);
		}
	}
	
	ofstream out(filename.c_str(), ofstream::out);
	try
	{
		out << root;
	}
	catch (...)
	{
		cerr <<"Error saving JSON settings file "+filename << endl;
		if (out.bad()) {
			cerr << "Problem with writing file" << endl;
		}
	}
}


void Params::setup()
{
	Lock lock(mMutex);

	mHasSetupBeenCalled = true;

	Vec2i size(350, 420);
	mParams = params::InterfaceGl("Params", Vec2i(size));
	mParams.setPosition(Vec2i(app::getWindowWidth() - size.x, 0));
	mParams.addButton("Save", std::bind((void (Params::*)())&Params::save, this));
	mParams.addButton("Save snapshot", std::bind(&Params::snapshot, this));
	vector<string> groups;
	for (int i=0; i<mParameters.size(); ++i)
	{
		mParameters[i]->readJson(mRoot);
	}
	for (auto it=mParameters.begin(); it!=mParameters.end(); ++it)
	{
		if ((**it).path.size()>1)
		{
			groups.push_back((**it).path[0]);
		}
		if ((**it).isSaved)
		{
			(**it).setup(mParams, [&]() { callbackParameterChanged(); });
		}
		else
		{
			(**it).setup(mParams);
		}
	}
	for (string const& group : groups)
	{
		mParams.setOptions(group, "opened=false");
	}
}

void Params::update(float dt, float elapsed)
{
	
}

//Json::Value Params::get(string const& path) const
//{
//	// for now, find final /
//	int s = path.find_last_of('/');
//	if (s==string::npos)
//		s = 0;
//	string base = path.substr(0, s);
//	string name = path.substr(s);
//	if (base!="")
//		return mRoot[base][name];
//	else
//		return mRoot[name];
//}


void Params::addParam(std::shared_ptr<BaseParameter> parameter)
{
	Lock lock(mMutex);

	mParameters.push_back(parameter);
	if (mHasSetupBeenCalled)
	{
		if (parameter->isSaved)
		{
			parameter->setup(mParams, [&]() { callbackParameterChanged(); });
			parameter->readJson(mRoot);
		}
		else
		{
			parameter->setup(mParams);
		}
	}
}

std::shared_ptr<BaseParameter> Params::addButton(std::function<void()> function, std::string const& name, std::vector<std::string> const& path, std::string const& options)
{
	Lock lock(mMutex);

	mParameters.push_back(std::shared_ptr<BaseParameter>(new Button(function, name, path, options)));
	return mParameters.back();
}

void Params::removeParam(std::shared_ptr<BaseParameter> parameter)
{
	Lock lock(mMutex);

	auto it = find(begin(mParameters), end(mParameters), parameter);
	while (it != end(mParameters))
	{
		mParameters.erase(it);
		it = find(begin(mParameters), end(mParameters), parameter);
	}
	parameter->removeFrom(mParams);
}


void Params::draw()
{
	Lock lock(mMutex);

	mParams.draw();
}




////////


void BaseParameter::writeJson(Json::Value& root) const
{
	Json::Value& child = getChild(root, path);
	toJson(child);
}


bool BaseParameter::readJson(Json::Value const& root)
{
	Json::Value const& child = getChild(root, path);
	return fromJson(child);
}





/////////

namespace Json
{
	// To avoid having to specialise the Parameter class
	bool operator>>(Json::Value const& child, double& value)
	{
		if (child.isNull() || !child.isConvertibleTo(Json::realValue))
			return false;
		value = child.asDouble();
		return true;
	}


	bool operator>>(Json::Value const& child, float& value)
	{
		if (child.isNull() || !child.isConvertibleTo(Json::realValue))
			return false;
		value = child.asFloat();
		return true;
	}
	
	bool operator>>(Json::Value const& child, int& value)
	{
		if (child.isNull() || !child.isConvertibleTo(Json::intValue))
			return false;
		value = child.asInt();
		return true;
	}
	
	bool operator>>(Json::Value const& child, ci::Vec2f& value)
	{
		if (!child.isNull() && child["x"].isConvertibleTo(Json::realValue)
			&& child["y"].isConvertibleTo(Json::realValue))
		{
			value.x = child["x"].asFloat();
			value.y = child["y"].asFloat();
			return true;
		}
		return false;
	}


	bool operator>>(Json::Value const& child, ci::Vec3f& value)
	{
		if (!child.isNull() && child["x"].isConvertibleTo(Json::realValue)
			&& child["y"].isConvertibleTo(Json::realValue)
			&& child["z"].isConvertibleTo(Json::realValue))
		{
			value.x = child["x"].asFloat();
			value.y = child["y"].asFloat();
			value.z = child["z"].asFloat();
			return true;
		}
		return false;
	}
	
	bool operator>>(Json::Value const& child, std::string& value)
	{
		if (child.isNull() || !child.isConvertibleTo(Json::stringValue))
			return false;
		value = child.asString();
		return true;
	}
	
	bool operator>>(Json::Value const& child, bool& value)
	{
		if (child.isNull() || !child.isConvertibleTo(Json::booleanValue))
			return false;
		value = child.asBool();
		return true;
	}

	Json::Value& operator<<(Json::Value& lhs, ci::Vec2f const& rhs)
	{
		lhs["x"] = rhs.x;
		lhs["y"] = rhs.y;
		return lhs;
	}
	
	Json::Value& operator<<(Json::Value& lhs, ci::Vec3f const& rhs)
	{
		lhs["x"] = rhs.x;
		lhs["y"] = rhs.y;
		lhs["z"] = rhs.z;
		return lhs;
	}
}



