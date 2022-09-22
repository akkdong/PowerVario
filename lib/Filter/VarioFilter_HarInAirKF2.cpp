// VarioFilter_HarInAirKF2.cpp
//

#include <Arduino.h>
#include "VarioFilter_HarInAirKF2.h"


#define KALMAN_UPDATE_FREQ          (25)




///////////////////////////////////////////////////////////////////////////////////////////////
//

VarioFilter_HarInAirKF2::VarioFilter_HarInAirKF2()
{
}


int VarioFilter_HarInAirKF2::begin(float zVariance, float zAccelVariance, float zAccelBiasVariance, float altitude)
{
	// init values
	zAccelVariance_ = zAccelVariance;
    zAccelBiasVariance_ = zAccelBiasVariance;
	zVariance_ = zVariance;

	//
	reset(altitude);

	return 0;
}

void VarioFilter_HarInAirKF2::update(float altitude, float va, float* altitudeFilteredPtr, float* varioPtr)
{
	// delta time
	#if 1
	uint32_t lastTick = millis();
	float dt = ((float)(lastTick - t_)) / 1000.0;
	t_ = lastTick;
	#else
	float dt = 1.0 / KALMAN_UPDATE_FREQ; // 25Hz
	#endif

	//
	// prediction
	//
	float accel = va - aBias_;
	v_ += accel * dt;
	z_ += v_ * dt;

	// Predict State Covariance matrix
	float t00,t01,t02;
	float t10,t11,t12;
	float t20,t21,t22;

	float dt2div2 = dt * dt / 2.0f;
	float dt3div2 = dt2div2 * dt;
	float dt4div4 = dt2div2 * dt2div2;

	t00 = Pzz_ + dt * Pvz_ - dt2div2 * Paz_;
	t01 = Pzv_ + dt * Pvv_ - dt2div2 * Pav_;
	t02 = Pza_ + dt * Pva_ - dt2div2 * Paa_;

	t10 = Pvz_ - dt * Paz_;
	t11 = Pvv_ - dt * Pav_;
	t12 = Pva_ - dt * Paa_;

	t20 = Paz_;
	t21 = Pav_;
	t22 = Paa_;

	Pzz_ = t00 + dt * t01 - dt2div2 * t02;
	Pzv_ = t01 - dt * t02;
	Pza_ = t02;

	Pvz_ = t10 + dt * t11 - dt2div2 * t12;
	Pvv_ = t11 - dt * t12;
	Pva_ = t12;

	Paz_ = t20 + dt * t21 - dt2div2 * t22;
	Pav_ = t21 - dt * t22;
	Paa_ = t22;

	Pzz_ += dt4div4 * zAccelVariance_;
	Pzv_ += dt3div2 * zAccelVariance_;

	Pvz_ += dt3div2 * zAccelVariance_;
	Pvv_ += dt * dt * zAccelVariance_;

	Paa_ += zAccelBiasVariance_;

	// Error
	float innov = altitude - z_;
	float sInv = 1.0f / (Pzz_ + zVariance_);

	// Kalman gains
	float kz = Pzz_ * sInv;
	float kv = Pvz_ * sInv;
	float ka = Paz_ * sInv;

	// Update state
	z_ += kz * innov;
	v_ += kv * innov;
	aBias_ += ka * innov;

	*altitudeFilteredPtr = z_;
	*varioPtr = v_;

	// Update state covariance matrix
	Paz_ -= ka * Pzz_;
	Pav_ -= ka * Pzv_;
	Paa_ -= ka * Pza_;

	Pvz_ -= kv * Pzz_;
	Pvv_ -= kv * Pzv_;
	Pva_ -= kv * Pza_;

	Pzz_ -= kz * Pzz_;
	Pzv_ -= kz * Pzv_;
	Pza_ -= kz * Pza_;
}

void VarioFilter_HarInAirKF2::reset(float altitude)
{
	z_ = altitude;
	v_ = 0.0f; // vInitial;
	t_ = millis();

	aBias_ = 0.0f; // aBiasInitial;

	Pzz_ = 1.0f;
	Pzv_ = 0.0f;
	Pza_ = 0.0f;

	Pvz_ = 0.0f;
	Pvv_ = 1.0f;
	Pva_ = 0.0f;

	Paz_ = 0.0f;
	Pav_ = 0.0;
	Paa_ = 100000.0f;
}
