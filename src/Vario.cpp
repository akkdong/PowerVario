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

#if 0
    Serial.print(pressure);
    Serial.print(", ");
    Serial.print(altitude);
    Serial.print(", ");
    Serial.print(altitudeFiltered);
    Serial.print(", ");
    Serial.println(vario);
#endif

    updateCount += 1;

    return 1;
}

int Vario::measure()
{
    static uint32_t lastTick = millis();
    uint32_t curTick = millis();

    if (curTick - lastTick < 1000 / 25)
        return -1;
    lastTick = curTick;

	temperature = sensor.getTemperature();	
	pressure = sensor.getPressure();
	
	return 0;
}
