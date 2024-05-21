#include <stdbool.h>
#include <stdio.h>
#define HS_DATA_PACKET_FULL_TIMESTAMP 0x0A0A
#define HS_DATA_PACKET_FULL_TIMESTAMP_V2 0x0A0C
#define GPS_DATA_PACKET 0x0A0D
#define GPS_PPS_PACKET 0x0A0E
typedef enum SensorType_e
    {
        Unknow = 0,
        Accel = 1,
        Gyro = 2,
        Mag = 3,
        Temperature = 4,
        Pressure = 5,
        Light = 6,
        Piezo=7,
        IMU=8
    }SensorType;

typedef struct DateTime_s {
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char weekDay;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
} DateTime;

typedef struct RAWXYZData_s
{
    unsigned long timeStamp;
    short X;
    short Y;
    short Z;
}RAWXYZData;
typedef struct RAWIMUData_s
{
    unsigned long timeStamp;
    short accelX;
    short accelY;
    short accelZ;

    short gyroX;
    short gyroY;
    short gyroZ;

    short magX;
    short magY;
    short magZ;
}RAWIMUData;
typedef struct IMUData_s
{
    double timeStamp;
    double accelX;
    double accelY;
    double accelZ;

    double gyroX;
    double gyroY;
    double gyroZ;

    double magX;
    double magY;
    double magZ;
}IMUData;

typedef struct SensorXYZData_s
{
    double timeStamp;
    double X;
    double Y;
    double Z;
}SensorXYZData;

typedef struct LightData_s
{
    double timeStamp;
    unsigned short ch0;
    unsigned short ch1;
}LightData;
typedef struct PressureData_s
{
    double timeStamp;
    double pressure;
}PressureData;

typedef struct TemperatureData_s
{
    double timeStamp;
    double temperature;
}TemperatureData;

typedef struct GPSDatas_s
{
    DateTime dateOfFix;
    bool fix;
    unsigned char fixQuality;
    double latitude;
    char latitudeDirection;
    double longitude;
    char longitudeDirection;
    double speed;
    double angle;
    double altitude;
    unsigned char satellites;
    unsigned char antenna;
}GPSDatas;


float GetFloatSafe(unsigned char *p, int index);
IMUData Normalize(RAWIMUData datas, unsigned char accelRange, unsigned char resolutionBits);
SensorXYZData NormalizeSensorsDatas(RAWXYZData datas, float range, unsigned char resolutionBits);
void ResetTimeStamp(void);
void ProcessDecodedMessage(short command, unsigned short payloadLength, unsigned char payload[], FILE* sensorsFile);