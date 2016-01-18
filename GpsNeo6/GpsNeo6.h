/**
 * Arduino - Gps driver
 *
 * NEO-6 implementation.
 *
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_GPS_NEO6_H__
#define __ARDUINO_DRIVER_GPS_NEO6_H__ 1

#include <Gps.h>

class GpsNeo6: public Gps {

public:


    /**
     * Gps API.
     */
    double getLatitude();
    double getLongitude();
    double getAltitude();
    double getSpeed();
    FixType getFixType();
    unsigned char getSatellites();
    double getCourseOverGround();
    double getVerticalSpeedIndicator();
    double getHorizontalDilutionOfPrecision();
    double getVerticalDilutionOfPrecision();
    unsigned char getYear();
    unsigned char getMonth();
    unsigned char getDay();
    unsigned char getHour();
    unsigned char getMinute();
    unsigned char getSecond();
};

#endif /* __ARDUINO_DRIVER_GPS_NEO6_H__ */
