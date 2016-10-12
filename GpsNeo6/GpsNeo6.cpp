#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "GpsNeo6.h"

GpsNeo6::GpsNeo6()
        : fieldNumber(0), fieldOffset(0), parity(0), messageType(GPS_NEO6_MESSAGE_INVALID), isChecksumField(false), latitude(0), longitude(0), altitude(0), speed(0), fixType(NO_FIX), satellites(0), courseOverGround(0), verticalSpeedIndicator(0), horizontalDilutionOfPrecision(0), verticalDilutionOfPrecision(
                0), year(0), month(0), day(0), hour(0), minute(0), second(0), locked(false) {
}

bool GpsNeo6::decode(unsigned char c) {
    bool ok = false;
    switch (c) {
    case ',':
        parity ^= c;
        /* no break */
    case '\r':
    case '\n':
    case '*':
        ok = parseField();
        isChecksumField = c == '*';
        break;
    case '$':
        initializeParser();
        break;
    default:
        if (fieldOffset < GPS_MEO6_MAX_FIELD_SIZE) {
            field[fieldOffset++] = c;
            parity ^= c;
            ok = true;
        }
    }
    return ok;
}

unsigned char GpsNeo6::decodeBytes(unsigned char *buf, unsigned char len) {
    unsigned char c = len;
    while (c-- > 0) {
        if (!decode(*buf++)) {
            return len - c;
        }
    }
    return len;
}

double GpsNeo6::GpsNeo6::getLatitude() {
    return latitude;
}

double GpsNeo6::GpsNeo6::getLongitude() {
    return longitude;
}

double GpsNeo6::GpsNeo6::getAltitude() {
    return altitude;
}

double GpsNeo6::GpsNeo6::getSpeed() {
    return speed;
}

GpsNeo6::FixType GpsNeo6::getFixType() {
    return fixType;
}

unsigned char GpsNeo6::getSatellites() {
    return satellites;
}

double GpsNeo6::GpsNeo6::getCourseOverGround() {
    return courseOverGround;
}

double GpsNeo6::GpsNeo6::getVerticalSpeedIndicator() {
    return verticalSpeedIndicator;
}

double GpsNeo6::GpsNeo6::getHorizontalDilutionOfPrecision() {
    return horizontalDilutionOfPrecision;
}

double GpsNeo6::GpsNeo6::getVerticalDilutionOfPrecision() {
    return verticalDilutionOfPrecision;
}

unsigned char GpsNeo6::getYear() {
    return year;
}

unsigned char GpsNeo6::getMonth() {
    return month;
}

unsigned char GpsNeo6::getDay() {
    return day;
}

unsigned char GpsNeo6::getHour() {
    return hour;
}

unsigned char GpsNeo6::getMinute() {
    return minute;
}

unsigned char GpsNeo6::getSecond() {
    return second;
}

unsigned char GpsNeo6::isLocked() {
    return locked;
}

void GpsNeo6::initializeParser() {
    messageType = GPS_NEO6_MESSAGE_INVALID;
    fieldNumber = 0;
    fieldOffset = 0;
    parity = 0;
}

bool GpsNeo6::parseField() {
    bool isFieldValid = false;
    if (fieldOffset < GPS_MEO6_MAX_FIELD_SIZE) {
        field[fieldOffset] = 0;
    }
    if (fieldNumber == 0) {
        isFieldValid = identifyMessage();
    } else {
        isFieldValid = parseIdentifiedMessage();
    }
    fieldNumber++;
    fieldOffset = 0;
    return isFieldValid;
}

bool GpsNeo6::isMessageValid() {
    return true;
}

bool GpsNeo6::parseIdentifiedMessage() {
    bool ok = false;
    switch (messageType) {
    case GPS_NEO6_MESSAGE_DTM:
        ok = parseDatumReference();
        break;
    case GPS_NEO6_MESSAGE_GBS:
        ok = parseSatelliteFaultDetection();
        break;
    case GPS_NEO6_MESSAGE_GGA:
        ok = parsePositioningSystemFix();
        break;
    case GPS_NEO6_MESSAGE_GLL:
        ok = parseLatitudeAndLongitude();
        break;
    case GPS_NEO6_MESSAGE_GPQ:
        ok = parsePollMessage();
        break;
    case GPS_NEO6_MESSAGE_GRS:
        ok = parseRangeResiduals();
        break;
    case GPS_NEO6_MESSAGE_GSA:
        ok = parseDOPAndActiveSatellites();
        break;
    case GPS_NEO6_MESSAGE_GST:
        ok = parsePseudoRangeErrorStatistics();
        break;
    case GPS_NEO6_MESSAGE_GSV:
        ok = parseSatellitesInView();
        break;
    case GPS_NEO6_MESSAGE_RMC:
        ok = parseRecommendedMinimumData();
        break;
    case GPS_NEO6_MESSAGE_TXT:
        ok = parseTextTransmission();
        break;
    case GPS_NEO6_MESSAGE_VTG:
        ok = parseCourseOverGroundAndGroundSpeed();
        break;
    case GPS_NEO6_MESSAGE_ZDA:
        ok = parseTimeAndDate();
        break;
    }
    return ok;
}

bool GpsNeo6::identifyMessage() {
    messageType = GPS_NEO6_MESSAGE_INVALID;
    if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GBS_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GBS;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GGA_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GGA;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GLL_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GLL;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GPQ_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GPQ;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GRS_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GRS;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GSA_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GSA;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GST_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GST;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_GSV_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_GSV;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_RMC_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_RMC;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_TXT_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_TXT;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_VTG_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_VTG;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_ZDA_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_ZDA;
    } else if (strcmp((const char *) field, GPS_NEO6_MESSAGE_DTM_START) == 0) {
        messageType = GPS_NEO6_MESSAGE_DTM;
    }
    return messageType != GPS_NEO6_MESSAGE_INVALID;
}

// $GPDTM,LLL,LSD,lat,N/S,lon,E/W,alt,RRR*cs<CR><LF>
bool GpsNeo6::parseDatumReference() {
    // Not implemented
    return false;
}

bool GpsNeo6::parseSatelliteFaultDetection() {
    // Not implemented
    return false;
}

bool GpsNeo6::parsePositioningSystemFix() {
    // Not implemented
    return false;
}

// $GPGLL,4717.11634,N,00833.91297,E,124923.00,A,A*6E
// $GPGLL,,,,,,V,N*64
bool GpsNeo6::parseLatitudeAndLongitude() {
    // Not implemented
    return false;
}

bool GpsNeo6::parsePollMessage() {
    // Not implemented
    return false;
}

bool GpsNeo6::parseRangeResiduals() {
    // Not implemented
    return false;
}

bool GpsNeo6::parseDOPAndActiveSatellites() {
    // Not implemented
    return false;
}

bool GpsNeo6::parsePseudoRangeErrorStatistics() {
    // Not implemented
    return false;
}

// $GPGSV,2,1,05,19,,,22,24,,,22,25,,,22,26,,,23*73
// $GPGSV,2,2,05,28,,,21*75
bool GpsNeo6::parseSatellitesInView() {
    bool ok = false;
    switch (fieldNumber) {
    case GPS_NEO6_MESSAGE_GSV_SATELLITES_IN_VIEW_FIELD:
        satellites = (unsigned char) atoi((const char *) field);
        ok = true;
        break;
    }
    return ok;
}

bool GpsNeo6::parseRecommendedMinimumData() {
    // Not implemented
    return false;
}

bool GpsNeo6::parseTextTransmission() {
    // Not implemented
    return false;
}

bool GpsNeo6::parseCourseOverGroundAndGroundSpeed() {
    // Not implemented
    return false;
}

bool GpsNeo6::parseTimeAndDate() {
    // Not implemented
    return false;
}
