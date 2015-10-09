#include "input/ControlPoint.h"
#include "Globals.h"
#include "Common.h"


using namespace std;
using namespace ci;




int ControlPoint::sNextId = 0;

bool ControlPoint::sEnableFilter;
float ControlPoint::sProcessNoisePos(.1f), ControlPoint::sProcessNoiseVel(1), ControlPoint::sProcessNoiseAcc(10), ControlPoint::sMeasurementNoise(0.1f);

int ControlPoint::sPersistence;
int ControlPoint::sReluctance;
float ControlPoint::sFilterAlpha;
float ControlPoint::sFilterBeta;
float ControlPoint::sTrendDecay;

// initial values set in PARAM macro
std::shared_ptr<float> const ControlPoint::sDynamicControlInputDecay(new float);
std::shared_ptr<float> const ControlPoint::sDynamicControlPeakDecay(new float);
std::shared_ptr<int> const ControlPoint::sDynamicControlWindowSize(new int);
std::shared_ptr<float> const ControlPoint::sDynamicControlScaleSpeed(new float);
std::shared_ptr<float> const ControlPoint::sDynamicControlScaleAccel(new float);

float ControlPoint::sScaleKalmanSpeed;


ControlPoint::ControlPoint(Vec2f const& pos2d_, Vec3f const& v, Vec2f const& centroid2d_)
	: pos2d(pos2d_)
	, pos(v)
	, speed(0)
	, id(sNextId++)
	, framesNotSeen(0)
	, framesSeen(0)
	, isActive(false)
	, filter(6, 2, 0, CV_32F)
	, birthTime(g.time)
	, centroid2d(centroid2d_)
	, centroid3d(gInput->mapKinectToWorld(centroid2d_, true))
	, distanceToCentroidSq(pos2d_.distanceSquared(centroid2d_))
	, accel(0)
	, dynamicSpeed(sDynamicControlInputDecay, sDynamicControlPeakDecay, sDynamicControlWindowSize, sDynamicControlScaleSpeed)
	, dynamicAccel(sDynamicControlInputDecay, sDynamicControlPeakDecay, sDynamicControlWindowSize, sDynamicControlScaleAccel)
	, dynamicSpeedGrapher("Dyn_spd_" + to_string(id), "ControlPointTracker", true, id)
	, dynamicAccelGrapher("Dyn_acc_" + to_string(id), "ControlPointTracker", true, id)
	, kalmanSpeedGrapher("Kal_spd_" + to_string(id), "ControlPointTracker", true, id)
	, dynamicControl(0)
{
	//filter.predictor.alpha = gInput->mControlPointFilterAlpha;
	//filter.predictor.beta = gInput->mControlPointFilterBeta;
	//filter.predictor.trendDecay = gInput->mControlPointTrendDecay;
	//filter.update(pos);
	updateTransitionMatrix();
	filter.statePre.at<float>(0) = pos2d_.x;
	filter.statePre.at<float>(1) = pos2d_.y;
	for (int i = 2; i < 6; i++)
	{
		filter.statePre.at<float>(i) = 0.f;
	}
	filter.statePre.copyTo(filter.statePost);
	cv::setIdentity(filter.measurementMatrix);
	//cv::setIdentity(filter.processNoiseCov, cv::Scalar::all(100.1f));
	//cv::setIdentity(filter.measurementNoiseCov, cv::Scalar::all(0.1f));
	cv::setIdentity(filter.errorCovPost, cv::Scalar::all(sMeasurementNoise));
}


/// Provide an observation of this control point
void ControlPoint::updateWithObservation(Vec2f const& pos2d_, Vec2f const& centroid2d_)
{
	Vec2f newPos;
	if (sEnableFilter && !gInput->isInputPaused)
	{
		updateTransitionMatrix();
		filter.predict();
		//cv::Mat measurement = *(cv::Mat_<float>(2, 1) << pos2d_.x, pos2d_.y);
		cv::Mat measurement(2, 1, CV_32F);
		measurement.at<float>(0) = pos2d_.x;
		measurement.at<float>(1) = pos2d_.y;
		cv::Mat corrected = filter.correct(measurement);
		//assert(corrected.size() == cv::Size(1, 4) && corrected.type() == CV_32F);
		newPos = Vec2f(corrected.at<float>(0), corrected.at<float>(1));
	}
	else
	{
		newPos = pos2d_;
	}

	if (!gInput->isInputPaused)
	{
		framesSeen++;
	}
	framesNotSeen = 0;
	centroid2d = centroid2d_;
	centroid3d = gInput->mapKinectToWorld(centroid2d, true);


	updateCommon(newPos);

}

/// Update control point if no observation found
void ControlPoint::updateWithoutObservation()
{
	updateTransitionMatrix();
	if (!gInput->isInputPaused)
	{
		framesNotSeen++;
		if (sEnableFilter && framesSeen>0)
		{
			//Vec2f newPos = predictPos2d();
			filter.predict();
			// correct filter with no measurement
			cv::Mat measurement(2, 1, CV_32F, cv::Scalar(0));
			cv::Mat measurementMatrix = filter.measurementMatrix;
			cv::Mat zeroMeasurementMatrix = cv::Mat::zeros(measurementMatrix.size(), measurementMatrix.type());
			filter.measurementMatrix = zeroMeasurementMatrix;
			cv::Mat corrected = filter.correct(measurement);
			filter.measurementMatrix = measurementMatrix;
			assert(cv::countNonZero(filter.measurementMatrix != zeroMeasurementMatrix) != 0);

			Vec2f newPos = Vec2f(corrected.at<float>(0), corrected.at<float>(1));

			updateCommon(newPos);



			//// manually move filter along
			//filter.statePre.copyTo(filter.statePost);
			//filter.errorCovPre.copyTo(filter.errorCovPost);
		}
	}
}

Vec2f ControlPoint::predictPos2d()
{
	// temp test
	return pos2d;
	if (sEnableFilter)
	{
		cv::Mat estimated = filter.predict();
		return Vec2f(estimated.at<float>(0), estimated.at<float>(1));
	}
	else
	{
		return pos2d + vel2d *  0.9f * float(g.dt);
	}
}

void ControlPoint::updateTransitionMatrix()
{
	//filter.transitionMatrix.at<float>(cv::Point(0, 2)) = g.dt;
	//filter.transitionMatrix.at<float>(cv::Point(1, 3)) = g.dt;
	//filter.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, g.dt, 0, 0, 1, 0, g.dt, 0, 0, 1, 0, 0, 0, 0, 1);
	//setIdentity(filter.transitionMatrix);
	//filter.transitionMatrix.at<float>(0, 2) = g.dt;
	//filter.transitionMatrix.at<float>(1, 3) = g.dt;
	//filter.transitionMatrix.at<
	//float transitionMatrixValues[6 * 6] = {
	//		1, 0, g.dt, 0, 0.5f*g.dt*g.dt, 0,
	//		0, 1, 0, g.dt, 0, 0.5*g.dt*g.dt,
	//		0, 0, 1, 0, g.dt, 0,
	//		0, 0, 0, 1, 0, g.dt,
	//		0, 0, 0, 0, 1, 0,
	//		0, 0, 0, 0, 0, 1
	//	};
	cv::Mat & A = filter.transitionMatrix;
	assert(A.size() == cv::Size(6, 6));

	cv::setIdentity(A);
	A.at<float>(0, 2) = A.at<float>(1, 3) = A.at<float>(2, 4) = A.at<float>(3, 5) = g.dt;
	A.at<float>(0, 4) = A.at<float>(1, 5) = g.dt * g.dt * 0.5f;

	cv::Mat & Q = filter.processNoiseCov;
	assert(Q.size() == cv::Size(6, 6));
	cv::setIdentity(Q, cv::Scalar::all(0));
	Q.at<float>(0, 0) = Q.at<float>(1, 1) = sProcessNoisePos;
	Q.at<float>(2, 2) = Q.at<float>(3, 3) = sProcessNoiseVel;
	Q.at<float>(4, 4) = Q.at<float>(5, 5) = sProcessNoiseAcc;

	cv::Mat & R = filter.measurementNoiseCov;
	assert(R.size() == cv::Size(2, 2));
	cv::setIdentity(R, cv::Scalar::all(sMeasurementNoise));
	//R.at<float>(0, 0) = R.at<float>(1, 1) = sMeasurementsNoise;


	//cv::Mat m = *(cv::Mat_<float>(6, 6) <<
	//	1, 0, g.dt, 0, 0.5f*g.dt*g.dt, 0,
	//	0, 1, 0, g.dt, 0, 0.5*g.dt*g.dt,
	//	0, 0, 1, 0, g.dt, 0,
	//	0, 0, 0, 1, 0, g.dt,
	//	0, 0, 0, 0, 1, 0,
	//	0, 0, 0, 0, 0, 1);
	//int t = m.type();
	//filter.transitionMatrix.setTo(*(cv::Mat_<float>(6,6) << 
	//	1, 0, g.dt, 0, 0.5f*g.dt*g.dt, 0,
	//	0, 1, 0, g.dt, 0, 0.5*g.dt*g.dt,
	//	0, 0, 1, 0, g.dt, 0,
	//	0, 0, 0, 1, 0, g.dt,
	//	0, 0, 0, 0, 1, 0,
	//	0, 0, 0, 0, 0, 1));

}

void ControlPoint::updateCommon(Vec2f const& newPos2d)
{
	Vec3f newPos = gInput->mapKinectToWorld(newPos2d, false);
	newPos.z = newPos.z == 0.f ? pos.z : newPos.z;
	if (!gInput->isInputPaused)
	{
		vel2d = (newPos2d - pos2d) / g.dt;
		vel = (newPos - pos) / g.dt;
		float z = centroid3d.z;// newPos.z;
		if (z < 0.01f)
		{
			z = pos.z;
		}
		if (z > 0.01f)
		{
			kalmanSpeed = sScaleKalmanSpeed * Vec2f(filter.statePost.at<float>(2), filter.statePost.at<float>(3)).length() * z;
			if (kalmanSpeed != kalmanSpeed)
			{
				int x = 0;
			}
			kalmanSpeedGrapher.update(kalmanSpeed);
		}
		//float newSpeed = vel.length();
		float newSpeed = kalmanSpeed;
		float newAccel = (newSpeed - speed) / g.dt;
		speed = newSpeed;
		accel = newAccel;
		dynamicSpeed.update(speed);
		dynamicAccel.update(accel);
		dynamicSpeedGrapher.update(dynamicSpeed.value);
		dynamicAccelGrapher.update(dynamicAccel.value);
		dynamicControl = dynamicSpeed.value;
	}
	pos = newPos;
	pos2d = newPos2d;
	distanceToCentroidSq = pos2d.distanceSquared(centroid2d);
	isActive = framesSeen >= sReluctance;
}

bool ControlPoint::isReadyForDeletion() const
{
	return framesNotSeen > sPersistence;
}

