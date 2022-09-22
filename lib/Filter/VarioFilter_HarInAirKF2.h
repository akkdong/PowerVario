// VarioFilter_HarInAirKF2.h
//

//
// Reference: https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/offline/kf/compare_kf2_kf3_kf4.ipynb
//
//
//

#ifndef __VARIL_FILTER_HARINAIRKF2_H__
#define __VARIL_FILTER_HARINAIRKF2_H__

#include "VarioFilter.h"


////////////////////////////////////////////////////////////////////////////////////////////
//

class VarioFilter_HarInAirKF2 : public IVarioFilter
{
public:
	VarioFilter_HarInAirKF2();

public:
	// IVarioFilter
	void					update(float altitude, float va, float* altitudeFiltered, float* vv);
	void					reset(float altitude);

	int						begin(float zVariance = 400.0, float zAccelVariance = 1000.0, float zAccelBiasVariance = 1.0, float altitude = 0);
	void					end();

	/*
	void					update(float altitude, float va, float* altitudeFilteredPtr, float* varioPtr);
	*/
	
protected:
	void					init();

private:
	// State being tracked
	float					z_;  // position
	float					v_;  // velocity
	float					aBias_;  // acceleration

	// 3x3 State Covariance matrix
	float					Pzz_;
	float					Pzv_;
	float					Pza_;
	float					Pvz_;
	float					Pvv_;
	float					Pva_;
	float					Paz_;
	float					Pav_;
	float					Paa_;
	float					zAccelBiasVariance_; // assumed fixed.
	float					zAccelVariance_;  // dynamic acceleration variance
	float					zVariance_; //  z measurement noise variance fixed

	uint32_t				t_;
};

#endif // __VARIL_FILTER_HARINAIRKF2_H__
