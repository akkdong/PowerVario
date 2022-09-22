// Common.h
//

#ifndef __COMMON_H__
#define __COMMON_H__

#define KALMAN_UPDATE_FREQ          (25)
#define NMEA_INTERVAL               (1000)

#define USE_FILTER_ROBIN_LILJA      (1)

#define PRESSURE_DAMPENING          (1.0 / KALMAN_UPDATE_FREQ / 2)
#define VARIO_DAMPENING             (1.0 / KALMAN_UPDATE_FREQ / 0.1)


#endif // __COMMON_H__
