#include "OscReceiver.h"
#include "Common.h"
#include <boost/range/adaptor/map.hpp>

using namespace ci;
using namespace std;

OscReceiver::OscReceiver(int port)
	: mMuteErrors(false)
	, mBuffer(16384, '\0')
	, mIncrementalRefreshCounter(0)
	, mSendSocket(nullptr)
{
	mListener.setup(port);
	mListener.registerMessageReceived([&](ci::osc::Message const* message) { callback(message); });
	pars.addParam(&mMuteErrors, "Mute OSC errors", { "Show" });
}


void OscReceiver::addMapping(OscMappingPtr mapping)
{
	assert(mapping != nullptr);
	if (mapping != nullptr)
	{
		mMappingsMap[mapping->widgetAddress] = mapping;
		mMappings.push_back(mapping);
	}
}

void OscReceiver::update()
{
	// invariant
	assert([this]() {
		for (OscMappingPtr const& m : mMappings)
		{
			if (mMappingsMap.count(m->widgetAddress) == 0)
				return false;
		}
		for (auto const& kv : mMappingsMap)
		{
			if (!isElem(mMappings, kv.second))
				return false;
			if (kv.first != kv.second->widgetAddress)
				return false;
		}
		return true;
	}());

	// check origin of any received messages
	if (!mQueue.empty())
	{
		std::string host = mQueue.front().getRemoteIp();
		if (host != mSendSocketHost)
		{
			mSendSocketHost = host;
			mSendSocket = make_unique<UdpTransmitSocket>(IpEndpointName(mSendSocketHost.c_str(), 2323));
			std::cout << "TouchOSC connected from " << mSendSocketHost << endl;
		}
	}

	// process received messages
	{
		boost::lock_guard<boost::mutex> lock(mQueueMutex);
		for (ci::osc::Message const& message : mQueue)
		{
			std::string address = message.getAddress();
			if (address == "/ping")
			{
				continue;
			}
			static std::string const multi("/multi");
			// addresses that start multi have their data included in the address
			// which means we need to strip it out when searching for the mapping
			if (address.find(multi) == 0)
			{
				// remove intiial /multi and trim to next slash. The + 1 is to go past the slash immediately following '/multi'
				address = address.substr(multi.size(), address.find_first_of('/', multi.size() + 1) - multi.size());
			}
			try
			{
				OscMappingPtr mapping = mMappingsMap.at(address);
				assert(mapping != nullptr);
				mapping->receive(message);
			}
			catch (std::out_of_range const&)
			{
				if (!mMuteErrors)
				{
					std::cout << "unrecognised osc message, address=" << address << " " << message.getNumArgs() << " args" << std::endl;
				}
			}
		}
		mQueue.clear();
	}

	// update touch osc state
	if (mSendSocket != nullptr && !mMappings.empty())
	{
		assert(!mSendSocketHost.empty());
		try
		{
			OutboundPacketStream out(mBuffer.data(), mBuffer.size());
			out << ::osc::BeginBundleImmediate;

			for (OscMappingPtr const& mapping : mMappings)
			{
				mapping->update(out);
			}

			mIncrementalRefreshCounter = (mIncrementalRefreshCounter + 1) % mMappings.size();
			mMappings[mIncrementalRefreshCounter]->incrementalRefresh(out);

			out << ::osc::EndBundle;
			mSendSocket->Send(out.Data(), out.Size());
		}
		catch (std::exception const& e)
		{
			if (!mMuteErrors)
			{
				std::cerr << "Error when sending data to TouchOSC: " << e.what() << std::endl;
			}
		}
		catch (...)
		{
			if (!mMuteErrors)
			{
				std::cerr << "Unanticipated error when sending data to TouchOSC." << std::endl;
			}
		}
	}
}


void OscReceiver::callback(ci::osc::Message const* message)
{
	boost::lock_guard<boost::mutex> lock(mQueueMutex);
	mQueue.push_back(*message);
}