#pragma once

#include "cinder/Vector.h"
#include "CinderOpenCv.h"


class Contour
{
public:
	/// Smoothed contour points
	std::vector<cv::Point> points;
	/// Original raw points
	std::vector<cv::Point> rawPoints;
	/// angle to go from point i-1 to point i.
	/// Only filled if Contour Shapes is enabled
	std::vector<double> angles;
	/// Convex, or mostly convex subcomponents
	/// Only filled if Contour Shapes is enabled and this contour is not itself a shape
	std::vector<Contour> shapes;
	/// If this contour is a subregional shape, then the apex is the starting point
	/// this indexes into points nad angles. Will be -1 otherwise
	size_t apexIndex;
	/// If this is a base contour then its area has been found
	double area;
	ci::Vec2f centroid;
	/// Region in original kinect image that this contour was found
	cv::Rect boundingBox;

	Contour()
		: apexIndex(-1)
		, area(0)
	{}
};
