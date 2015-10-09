#pragma once

#include "OscListener.h"
#include "OscMessage.h"
#include <deque>
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include <iostream>
#include <boost/thread.hpp>

using ::osc::OutboundPacketStream;

class OscMapping
{
public:
	OscMapping(std::string const& name_, int widgetId_)
		: name(name_)
		, widgetId(widgetId_)
		, labelAddress("/label/" + std::to_string(widgetId_))
		, widgetAddress("/" + std::to_string(widgetId_))
	{
		assert(std::string(name) != "");
	}

	/// reset all settings including label. called every so often
	virtual void incrementalRefresh(OutboundPacketStream & out)
	{
		send(out, labelAddress, name.c_str());
	}
	/// check if value has changed and update if necessary
	virtual void update(OutboundPacketStream & out) {}
	/// update value based on message
	virtual void receive(ci::osc::Message const& message) = 0;

	std::string const name;
	int const widgetId;

	std::string const labelAddress;
	std::string const widgetAddress;

protected:
	template <typename T>
	static void send(OutboundPacketStream & out, std::string const& address, T value)
	{
		out << ::osc::BeginMessage(address.c_str()) << value << ::osc::EndMessage;
	}
};
typedef std::shared_ptr<OscMapping> OscMappingPtr;


/// dummy mapping for just a label with no corresponding widget
class LabelMapping : public OscMapping
{
public:
	LabelMapping(std::string const& name_, int labelId)
		: OscMapping(name_, labelId)
	{
	}

	virtual void receive(ci::osc::Message const& message) override {}
};


class FaderMapping : public OscMapping
{
public:
	FaderMapping(std::string const& name, int widgetId, float* value_)
		: OscMapping(name, widgetId)
		, mValue(value_)
		, mPrevValue(*value_)
	{
		assert(mValue != nullptr);
	}

	virtual void incrementalRefresh(OutboundPacketStream & out) override
	{
		OscMapping::incrementalRefresh(out);
		send(out, widgetAddress, *mValue);
	}

	virtual void update(OutboundPacketStream & out) override
	{
		OscMapping::update(out);
		if (*mValue != mPrevValue)
		{
			send(out, widgetAddress, *mValue);
			mPrevValue = *mValue;
		}
	}

	virtual void receive(ci::osc::Message const& message) override
	{
		assert(message.getAddress() == widgetAddress);
		assert(message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT);
		if (message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT)
		{
			*mValue = mPrevValue = message.getArgAsFloat(0);
		}
	}

private:
	float* const mValue;
	float mPrevValue;
};


class ButtonMapping : public OscMapping
{
public:
	ButtonMapping(std::string const& name, int widgetId, std::function<void()> callback)
		: OscMapping(name, widgetId)
		, mCallback(callback)
	{}

	virtual void receive(ci::osc::Message const& message) override
	{
		assert(message.getAddress() == widgetAddress);
		assert(message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT);
		if (message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT)
		{
			int arg = message.getArgAsFloat(0);
			assert(arg == 0.f || arg == 1.f);
			if (arg != 0.f)
			{
				mCallback();
			}
		}
	}

private:
	std::function<void()> mCallback;
};


class ToggleMapping : public OscMapping
{
public:
	ToggleMapping(std::string const& name, int widgetId, bool* value)
		: OscMapping(name, widgetId)
		, mValue(value)
		, mPrevValue(*value)
	{
		assert(mValue != nullptr);
	}

	virtual void incrementalRefresh(OutboundPacketStream & out) override
	{
		OscMapping::incrementalRefresh(out);
		send(out, widgetAddress, *mValue ? 1 : 0);
	}

	virtual void update(OutboundPacketStream & out) override
	{
		OscMapping::update(out);
		if (*mValue != mPrevValue)
		{
			send(out, widgetAddress, *mValue ? 1 : 0);
			mPrevValue = *mValue;
		}
	}

	virtual void receive(ci::osc::Message const& message) override
	{
		assert(message.getAddress() == widgetAddress);
		assert(message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT);
		if (message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT)
		{
			float arg = message.getArgAsFloat(0);
			assert(arg == 0.f || arg == 1.f);
			*mValue = arg != 0.f;
		}
	}

private:
	bool* const mValue;
	bool mPrevValue;
};


class RowMultiToggleMapping : public OscMapping
{
public:
	RowMultiToggleMapping(std::string const& name, int widgetId, int* value)
		: OscMapping(name, widgetId)
		, mWidgetAddressWithRow("/multi"+widgetAddress + "/1/")
		, mValue(value)
		, mPrevValue(*value)
	{
		assert(mValue != nullptr);
	}

	std::string makeAddress(int value) const
	{
		// touch osc 1-indexes
		return mWidgetAddressWithRow + std::to_string(value+1);
	}

	virtual void incrementalRefresh(OutboundPacketStream& out) override
	{
		OscMapping::incrementalRefresh(out);
		send(out, makeAddress(*mValue), 1.f);
		
	}

	virtual void update(OutboundPacketStream& out) override
	{
		if (*mValue != mPrevValue)
		{
			incrementalRefresh(out);
			mPrevValue = *mValue;
		}
	}

	virtual void receive(ci::osc::Message const& message) override
	{
		std::string address = message.getAddress();
		assert(address.find(mWidgetAddressWithRow)==0);
		if (address.find(mWidgetAddressWithRow) == 0)
		{
			std::string scene = address.substr(mWidgetAddressWithRow.size());
			try
			{
				// touch osc 1-indexes values
				int sceneNumber = std::stoi(scene) - 1;
				assert(message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT);
				if (message.getNumArgs() == 1 && message.getArgType(0) == ci::osc::TYPE_FLOAT)
				{
					float arg = message.getArgAsFloat(0);
					if (arg != 0.f)
					{
						*mValue = sceneNumber;
					}
				}
			}
			catch (std::exception const& e)
			{
				std::cerr << "Invalid value received for " << widgetAddress << ": " << address << ". " << e.what() << std::endl;
			}
		}
		else
		{
			std::cerr << "No value detected for " << widgetAddress << ": " << address << std::endl;
		}
	}

private:
	std::string const mWidgetAddressWithRow;
	int* const mValue;
	int mPrevValue;
};


class OscReceiver
{
public:
	OscReceiver(int listenPort);
	void update();

	void addMapping(OscMappingPtr mapping);
	/// Takes ownership of \p mapping.
	void addMapping(OscMapping* mapping)
	{
		addMapping(OscMappingPtr(mapping));
	}

	bool mMuteErrors;

private:
	void callback(ci::osc::Message const* message);

	ci::osc::Listener mListener;
	std::unique_ptr<UdpTransmitSocket> mSendSocket;
	std::string mSendSocketHost;

	std::vector<char> mBuffer;

	boost::mutex mQueueMutex;
	std::vector<ci::osc::Message> mQueue;
	std::map<std::string, OscMappingPtr> mMappingsMap;
	std::vector<OscMappingPtr> mMappings;

	size_t mIncrementalRefreshCounter;
};