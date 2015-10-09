#pragma once

#include <boost/circular_buffer.hpp>

template <typename T>
class MovingPeak
{
public:
	MovingPeak();
	T update(T const& newValue);
	bool isEmpty() const;
	void clear();

	int numSamples;

private:
	boost::circular_buffer<T> mBuffer;
};

template <typename T>
MovingPeak<T>::MovingPeak()
: numSamples(40)
, mBuffer(40)
{}

template <typename T>
bool MovingPeak<T>::isEmpty() const
{
	return mBuffer.empty();
}


template <typename T>
void MovingPeak<T>::clear()
{
	mBuffer.clear();
}

template <typename T>
T MovingPeak<T>::update(T const& d)
{
	mBuffer.push_back(d);
	T m = mBuffer.front();
	for (auto it = cbegin(mBuffer) + 1; it != cend(mBuffer); ++it)
	{
		m = std::max(m, *it);
	}
	return m;
}