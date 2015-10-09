//
//  Thinner.h
//  TheMeasuresTaken
//
//  Created by Ali Nakipoglu on 6/21/13.
//
//

// Zhang&Suen http://www-prima.inrialpes.fr/perso/Tran/Draft/gateway.cfm.pdf
// Impl : http://answers.opencv.org/question/3207/what-is-a-good-thinning-algorithm-for-getting-the/

#pragma once

#include "opencv2/opencv.hpp"
#include "cinder/app/App.h"
//using namespace cv;

class Thinner
{

public:

	Thinner()
	{};

	~Thinner()
	{};

public:

	void static process(cv::Mat & inputarray, cv::Mat & outputarray);

private:

	void static ThinSubiteration1(cv::Mat & pSrc, cv::Mat & pDst);


	void static ThinSubiteration2(cv::Mat & pSrc, cv::Mat & pDst);
};
