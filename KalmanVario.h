// KalmanVario.h
//

#ifndef __KALMANVARIO_H__
#define __KALMANVARIO_H__

//
//
//

class KalmanFilter
{
public:
	KalmanFilter();

public:
	int						begin(float altitude, float zVariance = 400.0, float zAccelVariance = 1000.0, float zAccelBiasVariance = 1.0);
	void					end();

	void					update(float altitude, float va, float* altitudeFilteredPtr, float* varioPtr);
	
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
};

#endif // __KALMANVARIO_H__
