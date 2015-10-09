#pragma once

#include "cinder/Vector.h"
#include "utils/DynamicControlCalculator.h"
#include "ui/Grapher.h"


class ControlPoint
{
public:
	/// Pos in kinect space including any centroid-based transformations
	ci::Vec2f pos2d;
	/// Velocity in kinect space
	ci::Vec2f vel2d;
	/// Pos in world (camera) coordinates
	ci::Vec3f pos;
	/// World coordinates
	ci::Vec3f vel;
	/// World coordinates
	float speed;
	/// World coordinates
	float accel;
	int id;
	/// how many frames since this point was last seen
	int framesNotSeen;
	/// how many consecutive frames this point has been seen
	int framesSeen;
	/// Whether this control point should be considered active
	bool isActive;
	///// Filter object being applied to this control point
	//FilterSavitzky<ci::Vec3f> filter;
	cv::KalmanFilter filter;
	/// Time in seconds when this point was created
	float birthTime;
	/// Centroid of contour that this point derived from. In Kinect image coordinates
	ci::Vec2f centroid2d;
	/// In world coordinates
	ci::Vec3f centroid3d;
	/// Squared distance to centroid of originating user contour. In Kinect image coordinates. This
	/// is not affected by userScale based transformations
	float distanceToCentroidSq;

	// Special custom outputs
	/// Dynamic control from impossible alone, based on acceleration
	DynamicControlCalculator dynamicSpeed;
	DynamicControlCalculator dynamicAccel;
	/// This is based on the above two controls, at the moment it is speed.
	float dynamicControl;

	/// Speed according to the Kalman filter (Kinect coordinates, but scaled by static var)
	float kalmanSpeed;

	// Probably temp, to tweak constants
	Grapher dynamicSpeedGrapher;
	Grapher dynamicAccelGrapher;
	Grapher kalmanSpeedGrapher;


	static int sPersistence; ///< How many frames can pass without seeing before deleting
	static int sReluctance; ///< How many frames of consecutively seeing a point are needed before it is considered
	static float sFilterAlpha;
	static float sFilterBeta;
	static float sTrendDecay;

	static bool sEnableFilter;
	static float sProcessNoisePos, sProcessNoiseVel, sProcessNoiseAcc;
	static float sMeasurementNoise;

	static std::shared_ptr<float> const sDynamicControlInputDecay;
	static std::shared_ptr<float> const sDynamicControlPeakDecay;
	static std::shared_ptr<int> const sDynamicControlWindowSize;
	static std::shared_ptr<float> const sDynamicControlScaleAccel;
	static std::shared_ptr<float> const sDynamicControlScaleSpeed;
	
	static float sScaleKalmanSpeed;


	ControlPoint(ci::Vec2f const& pos2d_, ci::Vec3f const& v, ci::Vec2f const& centroid_);
	/// Provide an observation of this control point
	void updateWithObservation(ci::Vec2f const& pos2d_, ci::Vec2f const& centroid_);
	/// Update control point if no observation found
	void updateWithoutObservation();

	bool isReadyForDeletion() const;

	ci::Vec2f predictPos2d();

private:
	void updateTransitionMatrix();
	void updateCommon(ci::Vec2f const& newPos2d);

	static int sNextId;
};
