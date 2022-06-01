// Vario.cpp
//

#include "Vario.h"


Vario::Vario() : seaLevel(101325)
{
}


int Vario::begin()
{
	//
	sensor.begin(Bme280TwoWireAddress::Primary);
	sensor.setSettings(varioSettings());

	//
    for (int i = 0; i < 100; i ++)
    {
        delay(10);
        measure();
    }

    //
	updateCount = 0;
    altitudeFiltered = altitude;
    vario = 0.0;

	return 0;
}

void Vario::update()
{
    //
    measure();

    //
    if (updateCount < 100)
    {
        updateCount += 1;

        #if USE_FILTER_ROBIN_LILJA
        filter.Configure(30.0f, 4.0f, altitude);
        #else
        filter.begin(altitude, 400.0f, 1000.0f, 1.0f);
        #endif
    }
    else
    {
        #if USE_FILTER_ROBIN_LILJA
        filter.Update_Propagate(altitude, 0.0f, &altitudeFiltered, &vario);
        #else
        filter.update(altitude, 0.0f, &altitudeFiltered, &vario);
        #endif
    }
}

void Vario::measure()
{
	pressure = sensor.getPressure();
	temperature = sensor.getTemperature();	
	
	altitude = (1.0 - pow(pressure / seaLevel, 0.1902949572)) * (288.15 / 0.0065); // 4433076.0	
}
