#pragma once

#include <boost/circular_buffer.hpp>


template <typename T = float>
class PeakValue
{
public:
	PeakValue();

	T const& update(T const& nextValue)
	{
		buffer.push_back(nextValue);
		if (buffer.empty())
		{
			return nextValue;
		}
		auto it = buffer.begin();
		assert(it != buffer.end());
		T max = *(it++);
		for (; it != buffer.end(); ++it)
		{
			max = *it < max ? max : *it;
		}
		return max;
	}

	boost::circular_buffer<T> buffer;

};

