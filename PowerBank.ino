// PowerBank.ino
//

#include <Wire.h>
#include <Bme280.h>
#include "KalmanVario.h"

#include <stdarg.h>
#include <stdio.h>


#define RUN_MODE  0



//////////////////////////////////////////////////////////////////////////
//

// $LK8EX1,P,A,V,T,B,*checksum
//
// P : hPa * 100 (1013.25 hPa --> 101325), 999999(IGNORE)
// A : meter, 99999(IGNORE)
// V : cm/s, 9999(IGNORE)
// T : celsius (signed), 99(IGNORE)
// B : xx.x or percentage + 1000, 999(IGNORE)

char ToHexa(int value)
{
	if (value < 10)
		return '0' + value;
	
	return 'A' + (value - 10);
}

void makeSentence(char* buf, float p, float v, float t)
{
	int len = sprintf(buf, "$LK8EX1,%ld,99999,%d,%d,999,*xx\r\n", (long)(p), (int)(v * 100), (int)(t));
	
	const char* ptr = buf;
	uint8_t parity = *ptr;
	while (*ptr && *ptr != '*')
		parity ^= *ptr++;
	
	buf[len - 4] = ToHexa(parity >> 4);
	buf[len - 3] = ToHexa(parity & 0x0F);
}



//////////////////////////////////////////////////////////////////////////
//

#if 0
uint32_t lastTick;

void setup0()
{
	
	Serial.begin(9600);
	Wire.begin();
	delay(500);
	
	int ret = baro_init();
	Serial.print("baro.init = "); Serial.println(ret);
	// or
	//vario_init();
	
	lastTick = millis();
}

void loop0()
{
	if (millis() - lastTick > 1000)
	{
		baro_measure();
		//Printf("%.0f\n", baro_getAltitude() * 100);
		Serial.print("P = "); Serial.println(baro_getPressure());
		Serial.print("T = "); Serial.println(baro_getTemperature());
		Serial.print("A = "); Serial.println(baro_getAltitude());
		Serial.println("");
		
		lastTick = millis();
	}
	
	// or
	
	//flaot v = vario_update();
	//Serial.println("%.0f\n", v * 100);
}
#endif



KalmanVario 		vario;
uint32_t 			lastTick;
uint32_t 			updateCount;
volatile uint8_t	updateFlag;

uint8_t				ledState = 0;


void setTrigger();


void setup() 
{
	pinMode(13, OUTPUT);
	digitalWrite(13, ledState);

	Serial.begin(9600);
	Wire.begin();
	vario.begin();
	
	setTrigger();

	updateCount = 0;
	lastTick = millis();
}


void loop() 
{
	if (updateFlag != 0)
	{
		vario.update();
		
		updateCount++;
		updateFlag = 0;
	}

	/*
	auto temperature = String(sensor.getTemperature()) + " °C";
	auto pressure = String(sensor.getPressure() / 100.0) + " hPa";
	auto humidity = String(sensor.getHumidity()) + " %";

	String measurements = temperature + ", " + pressure + ", " + humidity;
	Serial.println(measurements);
	*/

	#if RUN_MODE != 2
	if (millis() - lastTick > 1000)
	#else
	if (millis() - lastTick > 100)
	#endif
	{
		#if RUN_MODE == 0
		char buf[64];
		makeSentence(buf, vario.getPressure(), vario.getVelocity(), vario.getTemperature());
		Serial.print(buf);
		#elif RUN_MODE == 1
		Serial.print("v = "); Serial.println(vario.getVelocity());
		Serial.print("p = "); Serial.print(vario.getPressure()); Serial.print(", "), Serial.println((long)vario.getPressure());
		Serial.print("t = "); Serial.println(vario.getTemperature());
		Serial.print("c = "); Serial.println(updateCount);
		#else // RUN_MODE = 2
		Serial.println(vario.getVelocity() * 100.0, 2);
		#endif
		
		updateCount = 0;
		lastTick += 1000;
		
		ledState = 1 - ledState;
		digitalWrite(13, ledState);
	}
}



#if defined(TCCR1A)

#define MAX_PERIOD 	0xFFFF

void findTopAndPrecaler(uint16_t freq, uint32_t& top, uint32_t& precaler)
{
	//
	precaler = 1;
	top = F_CPU / (freq * 1) - 1;
	
	if (top > MAX_PERIOD)
	{
		precaler = 2;
		top = F_CPU / (freq * 8) - 1;
		if (top > MAX_PERIOD)
		{
			precaler = 3;
			top = F_CPU / (freq * 64) - 1;
			if (top > MAX_PERIOD)
			{
				precaler = 4;
				top = F_CPU / (freq * 256) - 1;
				if (top > MAX_PERIOD)
				{
					precaler = 5;
					top = F_CPU / (freq * 1024) - 1;
				}
			}
		}
	}
}

void setTrigger()
{
	uint32_t top, precaler;
	findTopAndPrecaler(50, top, precaler);
	
	// precaler : 1, 8, 32, 64, 256, 1024
	// period count:
	// 8000000 / (20 *  1) = 400000
	// 8000000 / (20 *  8) = 50000
	// 8000000 / (20 * 32) = 12500
	// 8000000 / (20 * 64) = 6250
	// 
	
	// Fast PWM : TOV at TOP(OCRA)
	TCCR1A = 0b00000011;            // COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
	TCCR1B = 0b00011000 | precaler; // ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	OCR1A  = top;
	OCR1B  = top;
	TCNT1  = 0;
	TIMSK1 	= TIMSK1 | (1 << TOIE1);
	
	updateFlag = 0;
}



ISR(TIMER1_OVF_vect) 
{
	updateFlag = 1;
}

#endif