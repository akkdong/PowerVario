// VarioFilter_HarInAirKF4d.h
//

//
// Reference: https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/offline/kf/compare_kf2_kf3_kf4.ipynb
//
//
//

#ifndef __VARIL_FILTER_HARINAIRKF4D_H__
#define __VARIL_FILTER_HARINAIRKF4D_H__

#include "VarioFilter.h"


typedef struct KF4_STATE_ {
	float z; // altitude
	float v; // climb/sink rate
	float a; // gravity-compensated net earth-z axis acceleration
	float b; // acceleration residual bias (post-calibration)
} KF4_STATE;



////////////////////////////////////////////////////////////////////////////////////////////
//

class VarioFilter_HarInAirKF4d : public IVarioFilter
{
public:
	VarioFilter_HarInAirKF4d();

public:
	// IVarioFilter
	void            update(float altitude, float va, float* altitudeFiltered, float* vv);
	void            reset(float altitude);

	void            begin(float aVariance, float kAdapt, float zInitial, float vInitial = 1.0f, float aInitial = 1.0f);
	void            end();
	
protected:
    void            predict(float dt);
	void            init();

private:
    KF4_STATE       State;
    uint32_t        t_;

    // 4x4 process model state covariance estimate P 
    // Note : P is symmetric
    // Pzz Pzv Pza Pzb
    // Pvz Pvv Pva Pvb
    // Paz Pav Paa Pab
    // Pbz Pbv Pba Pbb

    float           Pzz;
    float           Pzv;
    float           Pza;
    float           Pzb;

    float           Pvz; 
    float           Pvv;
    float           Pva;
    float           Pvb;

    float           Paz; 
    float           Pav; 
    float           Paa;
    float           Pab;

    float           Pbz; 
    float           Pbv; 
    float           Pba; 
    float           Pbb;

    // 4x4 Process model noise covariance  Q
    // | 0  0  0     0     |
    // | 0  0  0     0     |
    // | 0  0  a_var 0     |
    // | 0  0  0     b_var |
    float           AccelVariance; // environmental acceleration variance, depends on conditions

    // acceleration bias noise variance
    float           ABiasVariance; 

    // measurement model sensor noise variance, measured offline
    float           ZMeasVariance; //  altitude measurement noise variance
    float           AMeasVariance; //  acceleration measurement noise variance

    // Adaptive uncertainty injection factor to allow for faster response in
    // high acceleration/deceleration situations
    float           KAdapt;
};

#endif // __VARIL_FILTER_HARINAIRKF4D_H__
