#include "Thinner.h"

void Thinner::process(cv::Mat & inputarray, cv::Mat & outputarray)
{
	bool bDone = false;
	int rows = inputarray.rows;
	int cols = inputarray.cols;

	inputarray.convertTo(inputarray, CV_8UC1);

	inputarray.copyTo(outputarray);

	outputarray.convertTo(outputarray, CV_8UC1);

	/// pad source
	cv::Mat p_enlarged_src = cv::Mat(rows + 2, cols + 2, CV_8UC1);
	for (int i = 0; i < (rows + 2); i++) {
		*p_enlarged_src.ptr<uint8_t>(i, 0) = uint8_t(0);
		*p_enlarged_src.ptr<uint8_t>(i, cols + 1) = uint8_t(0);
	}
	for (int j = 0; j < (cols + 2); j++) {
		*p_enlarged_src.ptr<uint8_t>(0, j) = uint8_t(0);
		*p_enlarged_src.ptr<uint8_t>(rows + 1, j) = uint8_t(0);
	}
	for (int i = 0; i < rows; i++) {
		uint8_t* in_row = inputarray.ptr<uint8_t>(i);
		uint8_t* out_row = p_enlarged_src.ptr<uint8_t>(i + 1);
		for (int j = 0; j < cols; j++) {
			out_row[j + 1] = uint8_t(in_row[j] > 0);
			//if (in_row[j] > 0) {
			//	//if (inputarray.at<uint8_t>(i, j) >= uint8_t(1)) {
			//	out_row[j + 1] = uint8_t(1);
			//	//p_enlarged_src.at<uint8_t>(i + 1, j + 1) = uint8_t(1);
			//}
			//else
			//	//p_enlarged_src.at<uint8_t>(i + 1, j + 1) = uint8_t(0);
			//	out_row[j + 1] = uint8_t(0);
		}
	}

	/// start to thin
	cv::Mat p_thinMat1 = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);
	cv::Mat p_thinMat2 = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);
	cv::Mat p_cmp = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);

	while (bDone != true) {
		/// sub-iteration 1
		ThinSubiteration1(p_enlarged_src, p_thinMat1);
		/// sub-iteration 2
		ThinSubiteration2(p_thinMat1, p_thinMat2);
		/// compare
		compare(p_enlarged_src, p_thinMat2, p_cmp, CV_CMP_EQ);
		/// check
		int num_non_zero = countNonZero(p_cmp);
		if (num_non_zero == (rows + 2) * (cols + 2)) {
			bDone = true;
		}
		/// copy
		p_thinMat2.copyTo(p_enlarged_src);
	}
	// copy result
	for (int i = 0; i < rows; i++) {
		uint8_t* out_row = outputarray.ptr<uint8_t>(i);
		uint8_t* in_row = p_enlarged_src.ptr<uint8_t>(i + 1);
		for (int j = 0; j < cols; j++) {
			//outputarray.at<uint8_t>(i, j) = p_enlarged_src.at<uint8_t>(i + 1, j + 1);
			out_row[j] = in_row[j + 1];
		}
	}
}


void Thinner::ThinSubiteration1(cv::Mat & pSrc, cv::Mat & pDst) {
	int rows = pSrc.rows;
	int cols = pSrc.cols;
	pSrc.copyTo(pDst);
	for (int i = 0; i < rows; i++) {
		uint8_t* Mi = pSrc.ptr<uint8_t>(i);
		for (int j = 0; j < cols; j++) {
			if (Mi[j] == uint8_t(1)) {
				//if (pSrc.at<uint8_t>(i, j) == uint8_t(1)) {
				/// get 8 neighbors
				assert(i>0 && i < rows - 1);
				assert(j>0 && j < cols - 1);
				uint8_t* MiMinusOne = pSrc.ptr<uint8_t>(i - 1);
				uint8_t* MiPlusOne = pSrc.ptr<uint8_t>(i + 1);
				uint8_t* const Rs[3] = { MiMinusOne, Mi, MiPlusOne };
				/// calculate C(p)
				//bool neighbor0 =  (bool)pSrc.at<uint8_t>(i - 1, j - 1);
				//bool neighbor1 =  (bool)pSrc.at<uint8_t>(i - 1, j);
				//bool neighbor2 =  (bool)pSrc.at<uint8_t>(i - 1, j + 1);
				//bool neighbor3 =  (bool)pSrc.at<uint8_t>(i, j + 1);
				//bool neighbor4 =  (bool)pSrc.at<uint8_t>(i + 1, j + 1);
				//bool neighbor5 =  (bool)pSrc.at<uint8_t>(i + 1, j);
				//bool neighbor6 =  (bool)pSrc.at<uint8_t>(i + 1, j - 1);
				//bool neighbor7 =  (bool)pSrc.at<uint8_t>(i, j - 1);
				bool neighbor0 = Rs[0][j - 1] > 0;// (int)pSrc.at<uint8_t>(i - 1, j - 1);
				bool neighbor1 = Rs[0][j] > 0;// (int)pSrc.at<uint8_t>(i - 1, j);
				bool neighbor2 = Rs[0][j + 1] > 0;// (int)pSrc.at<uint8_t>(i - 1, j + 1);
				bool neighbor3 = Rs[1][j + 1] > 0;// (int)pSrc.at<uint8_t>(i, j + 1);
				bool neighbor4 = Rs[2][j + 1] > 0;// (int)pSrc.at<uint8_t>(i + 1, j + 1);
				bool neighbor5 = Rs[2][j] > 0;// (int)pSrc.at<uint8_t>(i + 1, j);
				bool neighbor6 = Rs[2][j - 1] > 0;// (int)pSrc.at<uint8_t>(i + 1, j - 1);
				bool neighbor7 = Rs[1][j - 1] > 0;// (int)pSrc.at<uint8_t>(i, j - 1);
				uint8_t C = uint8_t(!neighbor1 && (neighbor2 || neighbor3)) +
					uint8_t(!neighbor3 && (neighbor4 || neighbor5)) +
					uint8_t(!neighbor5 && (neighbor6 || neighbor7)) +
					uint8_t(!neighbor7 && (neighbor0 || neighbor1));
				if (C == 1) {
					/// calculate N
					uint8_t N1 = uint8_t(neighbor0 || neighbor1) +
						uint8_t(neighbor2 || neighbor3) +
						uint8_t(neighbor4 || neighbor5) +
						uint8_t(neighbor6 || neighbor7);
					uint8_t N2 = uint8_t(neighbor1 || neighbor2) +
						uint8_t(neighbor3 || neighbor4) +
						uint8_t(neighbor5 || neighbor6) +
						uint8_t(neighbor7 || neighbor0);
					uint8_t N = MIN(N1, N2);
					if ((N == 2) || (N == 3)) {
						/// calculate criteria 3
						uint8_t c3 = (neighbor1 || neighbor2 || !neighbor4) && neighbor3;
						if (c3 == 0) {
							*pDst.ptr<uint8_t>(i, j) = uint8_t(0);
							//pDst.at<uint8_t>(i, j) = uint8_t(0);
						}
					}
				}
			}
		}
	}
}


void Thinner::ThinSubiteration2(cv::Mat & pSrc, cv::Mat & pDst) {
	int rows = pSrc.rows;
	int cols = pSrc.cols;
	pSrc.copyTo(pDst);
	for (int i = 0; i < rows; i++) {
		uint8_t* Mi = pSrc.ptr<uint8_t>(i);
		for (int j = 0; j < cols; j++) {
			if (Mi[j] == uint8_t(1)) {
				//if (pSrc.at<uint8_t>(i, j) == uint8_t(1)) {
				/// get 8 neighbors
				assert(i>0 && i < rows - 1);
				assert(j>0 && j < cols - 1);
				uint8_t* MiMinusOne = pSrc.ptr<uint8_t>(i - 1);
				uint8_t* MiPlusOne = pSrc.ptr<uint8_t>(i + 1);
				uint8_t* const Rs[3] = { MiMinusOne, Mi, MiPlusOne };
				/// calculate C(p)
				//int neighbor0 = (int)pSrc.at<uint8_t>(i - 1, j - 1);
				//int neighbor1 = (int)pSrc.at<uint8_t>(i - 1, j);
				//int neighbor2 = (int)pSrc.at<uint8_t>(i - 1, j + 1);
				//int neighbor3 = (int)pSrc.at<uint8_t>(i, j + 1);
				//int neighbor4 = (int)pSrc.at<uint8_t>(i + 1, j + 1);
				//int neighbor5 = (int)pSrc.at<uint8_t>(i + 1, j);
				//int neighbor6 = (int)pSrc.at<uint8_t>(i + 1, j - 1);
				//int neighbor7 = (int)pSrc.at<uint8_t>(i, j - 1);
				bool neighbor0 = Rs[0][j - 1] > 0;// (int)pSrc.at<uint8_t>(i - 1, j - 1);
				bool neighbor1 = Rs[0][j] > 0;// (int)pSrc.at<uint8_t>(i - 1, j);
				bool neighbor2 = Rs[0][j + 1] > 0;// (int)pSrc.at<uint8_t>(i - 1, j + 1);
				bool neighbor3 = Rs[1][j + 1] > 0;// (int)pSrc.at<uint8_t>(i, j + 1);
				bool neighbor4 = Rs[2][j + 1] > 0;// (int)pSrc.at<uint8_t>(i + 1, j + 1);
				bool neighbor5 = Rs[2][j] > 0;// (int)pSrc.at<uint8_t>(i + 1, j);
				bool neighbor6 = Rs[2][j - 1] > 0;// (int)pSrc.at<uint8_t>(i + 1, j - 1);
				bool neighbor7 = Rs[1][j - 1] > 0;// (int)pSrc.at<uint8_t>(i, j - 1);
				uint8_t C = uint8_t(!neighbor1 && (neighbor2 || neighbor3)) +
					uint8_t(!neighbor3 && (neighbor4 || neighbor5)) +
					uint8_t(!neighbor5 && (neighbor6 || neighbor7)) +
					uint8_t(!neighbor7 && (neighbor0 || neighbor1));
				if (C == 1) {
					/// calculate N
					uint8_t N1 = uint8_t(neighbor0 || neighbor1) +
						uint8_t(neighbor2 || neighbor3) +
						uint8_t(neighbor4 || neighbor5) +
						uint8_t(neighbor6 || neighbor7);
					uint8_t N2 = uint8_t(neighbor1 || neighbor2) +
						uint8_t(neighbor3 || neighbor4) +
						uint8_t(neighbor5 || neighbor6) +
						uint8_t(neighbor7 || neighbor0);
					uint8_t N = MIN(N1, N2);
					if ((N == 2) || (N == 3)) {
						uint8_t E = (neighbor5 || neighbor6 || !neighbor0) && neighbor7;
						if (E == 0) {
							*pDst.ptr<uint8_t>(i, j) = uint8_t(0);
							//pDst.at<uint8_t>(i, j) = uint8_t(0);
						}
					}
				}
			}
		}
	}
}