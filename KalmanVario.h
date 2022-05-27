// KalmanVario.h
//

#include "Bme280.h"


//
//
//

class KalmanVario
{
public:
	KalmanVario();

public:
	int						begin(float zVariance = 400.0, float zAccelVariance = 1000.0, float zAccelBiasVariance = 1.0);
	void					end();

	int						available();
	void					flush();

	void					update();

	float					getPressure() { return pressure; }
	float					getTemperature() { return temperature; }
	float					getAltitude2() { return z_; }
	float					getAltitude() { return altitude; }
	float					getCalibratedAltitude() { baroAltitude; }
	float					getVelocity() { return v_; }

	uint32_t				getTimestamp() { return t_; }

	void					calibrateAltitude(float altitudeRef);
	void					calculateSeaLevel(float altitude);
	
protected:
	static Bme280Settings vario() {
		return {
			.mode = Bme280Mode::Normal,
			.temperatureOversampling = Bme280Oversampling::X2,
			.pressureOversampling = Bme280Oversampling::X16,
			.humidityOversampling = Bme280Oversampling::Off,
			.filter = Bme280Filter::X16,
			.standbyTime = Bme280StandbyTime::Ms62_5			
		};
	  }
	  
	  void					measure();

protected:
//	static void IRAM_ATTR 	TimerProc();

	static KalmanVario *	mActiveVario;


private:
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

	// timestamp
	uint32_t				t_;

	// barometer altitude
	float					baroAltitude;
	// altitude calibration
	float					altitudeDrift;

	//
	int						varioUpdated;

	//
	Bme280TwoWire 			sensor;
	float					seaLevel;
	float					pressure;
	float					temperature;
	float					altitude;

protected:
//	hw_timer_t *			mTimer;
//	SemaphoreHandle_t		mSemaphore;
//	portMUX_TYPE			mMux;
};
