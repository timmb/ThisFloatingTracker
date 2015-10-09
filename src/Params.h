//
//  Params.h
//
//  Created by Tim Murray-Browne on 2014-04-06.
//
//

#pragma once

#include "json/json.h"
#include "cinder/params/Params.h"
#include "cinder/Cinder.h"
#include <boost/tokenizer.hpp>
#include <atomic>
#include "cinder/app/App.h"
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/lock_guard.hpp>


namespace Json
{
	// To avoid having to specialise the Parameter class
	bool operator>>(Json::Value const& child, double& value);
	bool operator>>(Json::Value const& child, float& value);
	bool operator>>(Json::Value const& child, int& value);
	bool operator>>(Json::Value const& child, ci::Vec2f& value);
	bool operator>>(Json::Value const& child, ci::Vec3f& value);
	bool operator>>(Json::Value const& child, std::string& value);
	bool operator>>(Json::Value const& child, bool& value);
	template <typename T>
	bool operator>>(Json::Value const& child, std::atomic<T>& value)
	{
		T t;
		if (child >> t)
		{
			value.store(t);
			return true;
		}
		return false;
	}

	template <typename T>
	Json::Value& operator<<(Json::Value& lhs, T const& rhs)
	{
		lhs = rhs;
		return lhs;
	}

	template <typename T>
	Json::Value& operator<<(Json::Value& lhs, std::atomic<T> const& rhs)
	{
		T t = rhs;
		return lhs << t;
	}
	Json::Value& operator<<(Json::Value& lhs, ci::Vec3f const& rhs);
	Json::Value& operator<<(Json::Value& lhs, ci::Vec2f const& rhs);
}


class BaseParameter
{
public:
	// basePath is list of parent nodes. root node first.
	// name should not be included in basePath
	BaseParameter(std::string const& name,
		std::vector<std::string> const& basePath = {},
		std::string const& options_ = "",
		std::function<void()> customCallback_ = nullptr)
		: path(basePath)
		, options(options_)
		, customCallback(customCallback_)
	{
		path.push_back(name);
		size_t unsavedPos = options.find("unsaved");
		isSaved = unsavedPos == std::string::npos;
		if (unsavedPos != std::string::npos)
		{
			options = options.substr(0, unsavedPos) + options.substr(unsavedPos + std::string("unsaved").size());
		}
	}

	virtual ~BaseParameter() {}

	virtual void setup(ci::params::InterfaceGl& params, std::function<void()> callbackFunction = nullptr) = 0;
	virtual void writeJson(Json::Value& root) const;
	virtual bool readJson(Json::Value const& root);

	void removeFrom(ci::params::InterfaceGl& params)
	{
		params.removeParam(mGuiName);
	}

	std::vector<std::string> path;
	std::string options;
	bool isSaved;
	std::function<void()> customCallback;
	/// Only used for int parameters. Leave blank for no labels
	std::vector<std::string> enumerationLabels;

protected:
	std::string mGuiName;
	template <typename JsonOrConstJson>
	JsonOrConstJson& getChild(JsonOrConstJson& root, std::vector<std::string> path) const;
	virtual void toJson(Json::Value& child) const = 0;
	virtual bool fromJson(Json::Value const& child) = 0;
};

// Main class to represent parameters
template <typename T>
class Parameter : public BaseParameter
{
public:
	Parameter(T* value_, std::string const& name, std::vector<std::string> const& basePath = {}, std::string const& options = "", std::function<void()> customCallback = nullptr)
		: BaseParameter(name, basePath, defaultOptions() + " " + options, customCallback)
		, value(value_)
		, mIsUpdateSignalConnected(false)
		, mPrevValue(-42)
	{}

	virtual ~Parameter()
	{
		//std::cout << " ~Parameter" << std::endl;
		cleanUpEnum();
	}

	virtual void setup(ci::params::InterfaceGl& params, std::function<void()> extraCallback_ = nullptr) override
	{
		std::string opts = options;
		mGuiName = path.back();
		if (path.size() > 1)
		{
			std::string group = path.front();
			std::replace(begin(group), end(group), ' ', '_');
			opts += " group=" + group;
			mGuiName += " (";
			for (int i = 0; i < path.size() - 2; i++)
			{
				mGuiName += path.at(i) + '/';
			}
			mGuiName += path.at(path.size() - 2) + ')';
		}
		extraCallback = extraCallback_;
		addToGui(params, std::move(opts));
	}

	void callbackFunction()
	{
		if (extraCallback != nullptr)
			extraCallback();
		if (customCallback != nullptr)
			customCallback();
	}

	std::string defaultOptions() const
	{
		return "";
	}



protected:
	void addToGui(ci::params::InterfaceGl& params, std::string optionsString)
	{
		params.addParam(mGuiName, value).optionsStr(optionsString).updateFn([this](){ callbackFunction(); });
	}

	virtual void toJson(Json::Value& child) const
	{
		child << *value;
	}

	virtual bool fromJson(Json::Value const& child)
	{
		T tempValue;
		if (child >> tempValue)
		{
			child >> *value;
			return true;
		}
		else
			return false;
	}

	T* value;
	std::function<void()> extraCallback;

	// For enumeration types not supporting a callback
	int mPrevValue;
	void updateEnum() {}
	void cleanUpEnum() {}
	boost::signals2::connection mUpdateSignalConnection;
	bool mIsUpdateSignalConnected;
};

template<>
std::string Parameter<float>::defaultOptions() const
{
	return "step=0.01 precision=6";
}


template<>
void Parameter<int>::updateEnum()
{
	if (*value != mPrevValue)
	{
		callbackFunction();
		mPrevValue = *value;
	}
}

template<>
void Parameter<int>::addToGui(ci::params::InterfaceGl& params, std::string optionsString)
{
	if (enumerationLabels.empty())
	{
		params.addParam(mGuiName, value).optionsStr(std::move(optionsString)).updateFn([this](){ callbackFunction(); });
	}
	else
	{
		params.addParam(mGuiName, enumerationLabels, value, std::move(optionsString));
		mUpdateSignalConnection = ci::app::App::get()->getSignalUpdate().connect(std::bind(&Parameter<int>::updateEnum, this));
		mIsUpdateSignalConnected = true;
		mPrevValue = *value;
	}
}

template<>
void Parameter<int>::cleanUpEnum()
{
	//std::cout << "Parameter<int>::cleanUpEnum" << std::endl;
	if (mIsUpdateSignalConnected)
	{
		mUpdateSignalConnection.disconnect();
		mIsUpdateSignalConnected = false;
	}
}





// Class to represent buttons
class Button : public BaseParameter
{
public:
	Button(std::function<void()> function_, std::string const& name, std::vector<std::string> const& basePath = {}, std::string const& options = "")
		: BaseParameter(name, basePath, options)
		, function(function_)
	{
		isSaved = false;
	}

	virtual void setup(ci::params::InterfaceGl& params, std::function<void()> ignored = nullptr) override
	{
		std::string opts = options;
		std::string guiName = path.back();
		if (path.size() > 1)
		{
			opts += " group=" + path.front();
			guiName += " (";
			for (int i = 0; i < path.size() - 2; i++)
			{
				guiName += path.at(i) + '/';
			}
			guiName += path.at(path.size() - 2) + ')';
		}
		params.addButton(guiName, function, opts);
	}

protected:
	virtual void toJson(Json::Value& child) const
	{
	}

	virtual bool fromJson(Json::Value const& child)
	{
		return true;
	}

	std::function<void()> function;
};





class Params
{
public:
	Params();
	virtual ~Params();

	/// Effectively destroys this instance
	void clear();

	bool load(std::string const& jsonFile);
	void setup();
	void save();
	void save(std::string const& filename);
	void snapshot();
	void update(float dt, float elapsedTime);
	void draw();

	void addParam(std::shared_ptr<BaseParameter> parameter);
	/// Takes ownership of \p parameter
	std::shared_ptr<BaseParameter> addParam(BaseParameter* parameter)
	{
		auto p = std::shared_ptr<BaseParameter>(parameter);
		addParam(p);
		return p;
	}

	void removeParam(std::shared_ptr<BaseParameter> parameter);

	template <typename T>
	std::shared_ptr<Parameter<T>> addParam(T* value, std::string const& name, std::vector<std::string> const& path = {}, std::string const& options = "");

	std::shared_ptr<BaseParameter> addButton(std::function<void()> function, std::string const& name, std::vector<std::string> const& path = {}, std::string const& options = "");

	/// path is slash separated list of key names, e.g.
	/// "/key1/key2".
	//	Json::Value get(std::string const& basepath) const;

	// callback for all saved parameters, triggers a save on this parameter
	void callbackParameterChanged()
	{
		save();
	}


private:
	typedef boost::lock_guard<boost::recursive_mutex> Lock;

	ci::params::InterfaceGl mParams;
	std::string mJsonFile;
	Json::Value mRoot;

	std::vector<std::shared_ptr<BaseParameter> > mParameters;
	bool mHasSetupBeenCalled;

	boost::recursive_mutex mMutex;

};


template <typename T>
std::shared_ptr<Parameter<T>> Params::addParam(T* value, std::string const& name, std::vector<std::string> const& path, std::string const& options)
{
	std::shared_ptr<Parameter<T>> p(new Parameter<T>(value, name, path, options));
	addParam(p);
	return p;

}

/// recursive find json from path
template <typename T>
T& findJsonFromPath(T& root, std::vector<std::string> path)
{
}


template <typename T>
T& BaseParameter::getChild(T& root, std::vector<std::string> path) const
{
	if (path.empty())
	{
		return root;
	}
	return getChild(root[path.front()], std::vector<std::string>(path.begin() + 1, path.end()));
}


typedef std::shared_ptr<BaseParameter> ParameterPtr;
template <typename T>
using ParameterPtrT = std::shared_ptr<Parameter<T>>;