// Vario.h
//

#include "Common.h"
#include "Bme280.h"
#if USE_FILTER_ROBIN_LILJA
#include "KalmanFilter.h"
#else
#include "KalmanVario.h"
#endif

//
//
//

class Vario
{
public:
	Vario();

public:
	int						begin();
	void					end();

	int						available() { return updateCount < 100 ? 0 : 1; }
	void					update();

	float					getPressure() { return pressure; }
	float					getTemperature() { return temperature; }
	float					getAltitudeFiltered() { return altitudeFiltered; }
	float					getAltitude() { return altitude; }
	float					getVelocity() { return vario; }

    #if 0
	void					calibrateAltitude(float altitudeRef);
	void					calculateSeaLevel(float altitude);
    #endif
	
protected:
	static Bme280Settings varioSettings() {
		return {
			.mode = Bme280Mode::Normal,
			.temperatureOversampling = Bme280Oversampling::X2,
			.pressureOversampling = Bme280Oversampling::X16,
			.humidityOversampling = Bme280Oversampling::Off,
			.filter = Bme280Filter::X16,
			.standbyTime = Bme280StandbyTime::Ms0_5			
		};
	  }
	  
	  void					measure();

protected:
    //
	static Vario *	        mActiveVario;

	// sensor
	Bme280TwoWire 			sensor;
	float					seaLevel;
	float					pressure;
	float					temperature;
	float					altitude;

    // filter
    #if USE_FILTER_ROBIN_LILJA
    KalmanFilter            filter;
    #else
    KalmanFilter            filter;
    #endif

    // vario
	int						updateCount;
    
    float                   altitudeFiltered;
    float                   vario;
};
