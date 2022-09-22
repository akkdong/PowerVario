// Vario.cpp
//

#include "Vario.h"


Vario::Vario() : seaLevel(101325)
{
}


int Vario::begin(IVarioFilter* _filter)
{
    //
    filter = _filter;

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

int Vario::update()
{
    //
    if (measure() < 0)
        return -1;

    altitude = (1.0 - pow(pressure / seaLevel, 1 / 5.25579)) * ((temperature + 273.15) / 0.0065);

    //
    if (updateCount == 0)
        filter->reset(altitude);
    else
        filter->update(altitude, 0, &altitudeFiltered, &vario);

    /*
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
    */
}

int Vario::measure()
{
    static uint32_t lastTick = millis();
    uint32_t curTick = millis();

    if (curTick - lastTick < 1000 / 25)
        return -1;
    lastTick = curTick;

	pressure = sensor.getPressure();
	temperature = sensor.getTemperature();	
	
	return 0;
}
