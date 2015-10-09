#pragma once

#include <boost/circular_buffer.hpp>

template <typename T>
class MovingTrough
{
public:
	MovingTrough();
	T update(T const& newValue);
	bool isEmpty() const;
	void clear();

	int numSamples;

private:
	boost::circular_buffer<T> mBuffer;
};

template <typename T>
MovingTrough<T>::MovingTrough()
: numSamples(40)
, mBuffer(40)
{}

template <typename T>
bool MovingTrough<T>::isEmpty() const
{
	return mBuffer.empty();
}


template <typename T>
void MovingTrough<T>::clear()
{
	mBuffer.clear();
}

template <typename T>
T MovingTrough<T>::update(T const& d)
{
	mBuffer.push_back(d);
	T m = mBuffer.front();
	for (auto it = cbegin(mBuffer) + 1; it != cend(mBuffer); ++it)
	{
		m = std::min(m, *it);
	}
	return m;
}