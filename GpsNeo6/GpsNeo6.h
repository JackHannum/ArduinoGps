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

#define GPS_MEO6_MAX_FIELD_SIZE 16

#define GPS_NEO6_MESSAGE_GBS_START "GPGBS"
#define GPS_NEO6_MESSAGE_GGA_START "GPGGA"
#define GPS_NEO6_MESSAGE_GLL_START "GPGLL"
#define GPS_NEO6_MESSAGE_GPQ_START "GPGPQ"
#define GPS_NEO6_MESSAGE_GRS_START "GPGRS"
#define GPS_NEO6_MESSAGE_GSA_START "GPGSA"
#define GPS_NEO6_MESSAGE_GST_START "GPGST"
#define GPS_NEO6_MESSAGE_GSV_START "GPGSV"
#define GPS_NEO6_MESSAGE_RMC_START "GPRMC"
#define GPS_NEO6_MESSAGE_TXT_START "GPTXT"
#define GPS_NEO6_MESSAGE_VTG_START "GPVTG"
#define GPS_NEO6_MESSAGE_ZDA_START "GPZDA"
#define GPS_NEO6_MESSAGE_DTM_START "GPDTM"

#define GPS_NEO6_MESSAGE_GSV_SATELLITES_IN_VIEW_FIELD   3

#define JOIN(messageType, fieldNumber) (unsigned char)(((unsigned char)(messageType) << 4) | (fieldNumber & 0x0f))

class GpsNeo6: public Gps {

    unsigned char field[GPS_MEO6_MAX_FIELD_SIZE];
    unsigned char fieldNumber;
    unsigned char fieldOffset;
    unsigned char parity;
    unsigned char messageType;
    bool isChecksumField;

    double latitude;
    double longitude;
    double altitude;
    double speed;
    FixType fixType;
    unsigned char satellites;
    double courseOverGround;
    double verticalSpeedIndicator;
    double horizontalDilutionOfPrecision;
    double verticalDilutionOfPrecision;
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char locked;

public:

    enum MessageType {

        // Datum Reference
        GPS_NEO6_MESSAGE_INVALID = 0x00,

        // GNSS Satellite Fault Detection
        GPS_NEO6_MESSAGE_GBS = 0x01,

        // Global positioning system fix data
        GPS_NEO6_MESSAGE_GGA = 0x02,

        // Latitude and longitude, with time of position fix and status
        GPS_NEO6_MESSAGE_GLL = 0x03,

        // Poll message
        GPS_NEO6_MESSAGE_GPQ = 0x04,

        // GNSS Range Residuals
        GPS_NEO6_MESSAGE_GRS = 0x05,

        // GNSS DOP and Active Satellites
        GPS_NEO6_MESSAGE_GSA = 0x06,

        // GNSS Pseudo Range Error Statistics
        GPS_NEO6_MESSAGE_GST = 0x07,

        // GNSS Satellites in View
        GPS_NEO6_MESSAGE_GSV = 0x08,

        // Recommended Minimum data
        GPS_NEO6_MESSAGE_RMC = 0x09,

        // Text Transmission
        GPS_NEO6_MESSAGE_TXT = 0x0a,

        // Course over ground and Ground speed
        GPS_NEO6_MESSAGE_VTG = 0x0b,

        // Time and Date
        GPS_NEO6_MESSAGE_ZDA = 0x0c,

        // Datum Reference
        GPS_NEO6_MESSAGE_DTM = 0x0d
    };

    GpsNeo6();

    bool decode(unsigned char c);

    unsigned char decodeBytes(unsigned char *buf, unsigned char len);

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
    unsigned char isLocked();

private:

    void initializeParser();
    bool parseField();
    bool isMessageValid();
    bool parseIdentifiedMessage();
    bool identifyMessage();

    /**
     * Datum Reference
     * This message gives the difference between the currently selected Datum, and the reference Datum.
     * If the currently configured Datum is not WGS84 or WGS72, then the field LLL will be set to 999,
     * and the field LSD is set to a variable-length string, representing the Name of the Datum.
     * The list of supported datums can be found in CFG-DAT.
     * The reference Datum can not be changed and is always set to WGS
     */
    bool parseDatumReference();

    /**
     * GNSS Satellite Fault Detection
     *
     * This message outputs the results of the Receiver Autonomous Integrity Monitoring Algorithm (RAIM).
     * - The fields errlat, errlon and erralt output the standard deviation of the position calculation,
     * using all satellites which pass the RAIM test successfully.
     * - The fields errlat, errlon and erralt are only output if the RAIM process passed successfully
     * (i.e. no or successful Edits happened). These fields are never output if 4 or fewer satellites are used
     * for the navigation calculation (because - in this case - integrity can not be determined by the receiver autonomously)
     * - The fields prob, bias and stdev are only output if at least one satellite failed in the RAIM test.
     * If more than one satellites fail the RAIM test, only the information for the worst satellite is output in this message.
     */
    bool parseSatelliteFaultDetection();

    /**
     * Global positioning system fix data
     *
     * The output of this message is dependent on the currently selected datum (Default: WGS84)
     * Time and position, together with GPS fixing related data (number of satellites in use, and
     * the resulting HDOP, age of differential data if in use, etc.).
     */
    bool parsePositioningSystemFix();

    /**
     * Latitude and longitude, with time of position fix and status
     */
    bool parseLatitudeAndLongitude();

    /**
     * Poll message
     */
    bool parsePollMessage();

    /**
     * GNSS Range Residuals
     *
     * This messages relates to associated GGA and GSA messages.
     * If less than 12 SVs are available, the remaining fields are output empty. If more than 12 SVs are used,
     * only the residuals of the first 12 SVs are output, in order to remain consistent with the NMEA standard.
     */
    bool parseRangeResiduals();

    /**
     * GNSS DOP and Active Satellites
     *
     * The GPS receiver operating mode, satellites used for navigation, and DOP values.
     * - If less than 12 SVs are used for navigation, the remaining fields are left empty.
     * If more than 12 SVs are used for navigation, only the IDs of the first 12 are output.
     * - The SV Numbers (Fields 'Sv') are in the range of 1 to 32 for GPS satellites,
     * and 33 to 64 for SBAS satellites (33 = SBAS PRN 120, 34 = SBAS PRN 121, and so on)
     */
    bool parseDOPAndActiveSatellites();

    /**
     * GNSS Pseudo Range Error Statistics
     */
    bool parsePseudoRangeErrorStatistics();

    /**
     * GNSS Satellites in View
     *
     * The number of satellites in view, together with each PRN (SV ID), elevation and azimuth,
     * and C/No (Signal/Noise Ratio) value. Only four satellite details are transmitted in one message.
     */
    bool parseSatellitesInView();

    /**
     * Recommended Minimum data
     *
     * The output of this message is dependent on the currently selected datum (Default: WGS84)
     * The Recommended Minimum sentence defined by NMEA for GPS/Transit system data.
     */
    bool parseRecommendedMinimumData();

    /**
     * Text Transmission
     *
     * This message is not configured through CFG-MSG, but instead through CFG-INF.
     * This message outputs various information on the receiver, such as power-up screen,
     * software version etc. This message can be configured using UBX Protocol message CFG-INF
     */
    bool parseTextTransmission();

    /**
     * Course over ground and Ground speed
     *
     * Velocity is given as Course over Ground (COG) and Speed over Ground (SOG).
     */
    bool parseCourseOverGroundAndGroundSpeed();

    /**
     * Time and Date
     */
    bool parseTimeAndDate();
};

#endif /* __ARDUINO_DRIVER_GPS_NEO6_H__ */
