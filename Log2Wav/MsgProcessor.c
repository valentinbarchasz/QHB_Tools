 #include <stdbool.h>
 #include <math.h>
 #include <stdio.h>
 #include "MsgProcessor.h"
 #include "Macros.h"
 
float GetFloatSafe(unsigned char *p, int index)
{
    unsigned char tmp[4];
    tmp[0]=p[0+index];
    tmp[1]=p[1+index];
    tmp[2]=p[2+index];
    tmp[3]=p[3+index];
    float *result_ptr = (float*)(tmp);
    float result = *result_ptr;
    return result;
}

IMUData Normalize(RAWIMUData datas, unsigned char accelRange, unsigned char resolutionBits)
{
    IMUData dataNormalized ;
    double accelMaxValue = pow(2, resolutionBits)/2;
    double gyroMaxValue = pow(2, resolutionBits) / 2;
    double gyroRange = 250.0;       //Hardcode pour le moment
    double magRange = 4900.0;       //Fixe

    dataNormalized.timeStamp = (double)(datas.timeStamp)/1000.0;

    dataNormalized.accelX = accelRange / accelMaxValue * datas.accelX;
    dataNormalized.accelY = accelRange / accelMaxValue * datas.accelY;
    dataNormalized.accelZ = accelRange / accelMaxValue * datas.accelZ;

    dataNormalized.gyroX = ((datas.gyroX > gyroMaxValue) ? (double)gyroRange / gyroMaxValue * ((double)datas.gyroX - 2 * gyroMaxValue) : gyroRange / gyroMaxValue * datas.gyroX);
    dataNormalized.gyroY = ((datas.gyroY > gyroMaxValue) ? (double)gyroRange / gyroMaxValue * ((double)datas.gyroY - 2 * gyroMaxValue) : gyroRange / gyroMaxValue * datas.gyroY);
    dataNormalized.gyroZ = ((datas.gyroZ > gyroMaxValue) ? (double)gyroRange / gyroMaxValue * ((double)datas.gyroZ - 2 * gyroMaxValue) : gyroRange / gyroMaxValue * datas.gyroZ);

    dataNormalized.magX = ((datas.magX > gyroMaxValue) ? (double)magRange / gyroMaxValue * ((double)datas.magX - 2 * gyroMaxValue) : magRange / gyroMaxValue * datas.magX); 
    dataNormalized.magY = ((datas.magY > gyroMaxValue) ? (double)magRange / gyroMaxValue * ((double)datas.magY - 2 * gyroMaxValue) : magRange / gyroMaxValue * datas.magY); 
    dataNormalized.magZ = ((datas.magZ > gyroMaxValue) ? (double)magRange / gyroMaxValue * ((double)datas.magZ - 2 * gyroMaxValue) : magRange / gyroMaxValue * datas.magZ); 
    return dataNormalized;
}

SensorXYZData NormalizeSensorsDatas(RAWXYZData datas, float range, unsigned char resolutionBits)
{
    SensorXYZData dataNormalized;
    double dataMaxValue = pow(2, resolutionBits) / 2;

    //TimeStamp en mS
    dataNormalized.timeStamp = (double)datas.timeStamp / 1000.0;

    dataNormalized.X = range / dataMaxValue * datas.X;
    dataNormalized.Y = range / dataMaxValue * datas.Y;
    dataNormalized.Z = range / dataMaxValue * datas.Z;
    return dataNormalized;
}


//Une fois processé, le message sera transformé en event sortant
unsigned int lastAccelTimeStamp;
unsigned int lastGyroTimeStamp;
unsigned int lastMagTimeStamp;
unsigned int lastLightTimeStamp;
unsigned int lastPressureTimeStamp;
unsigned int lastTemperatureTimeStamp;
unsigned int lastTimeStamp;
DateTime lastGPSDate;
double lastPPSTimeStampNS;

void ResetTimeStamp(void)
{
    lastAccelTimeStamp = 0;
    lastGyroTimeStamp = 0;
    lastMagTimeStamp = 0;
    lastPressureTimeStamp = 0;
    lastTemperatureTimeStamp = 0;
    lastLightTimeStamp=0;
}

void ProcessDecodedMessage(short command, unsigned short payloadLength, unsigned char payload[], FILE* sensorsFile)
{
        unsigned int timeStamp = 0;
        switch (command)
        {
            case (short)HS_DATA_PACKET_FULL_TIMESTAMP:
                {
                    SensorType type = (SensorType)payload[0];
                    unsigned char id = payload[1];
                    unsigned char nbChannels = payload[2];
                    unsigned char range = payload[3];
                    unsigned char resolutionBits = payload[4];
                    unsigned short samplingFrequency = BUILD_UINT16(payload[6], payload[5]);
                    unsigned short nbSamples = BUILD_UINT16(payload[8], payload[7]);

                    int lengthPerSample = nbChannels * resolutionBits / 8 + 4;
                    for (int i = 0; i < nbSamples && payloadLength >= lengthPerSample * i + 9; i++)
                    {
                        timeStamp = BUILD_UINT32(9 + i * lengthPerSample,9 + i * lengthPerSample+1,9 + i * lengthPerSample+2,9 + i * lengthPerSample+3);

                        if (timeStamp > lastTimeStamp)
                        {
                            lastTimeStamp = timeStamp;
                            switch (type)
                            {
                                case IMU:
                                {
                                    RAWIMUData data;
                                    data.timeStamp = timeStamp;
                                    data.accelX = BUILD_UINT16(13 + i * lengthPerSample,13 + i * lengthPerSample+1);
                                    data.accelY = BUILD_UINT16(15 + i * lengthPerSample,15 + i * lengthPerSample+1);
                                    data.accelZ = BUILD_UINT16(17 + i * lengthPerSample,17 + i * lengthPerSample+1);

                                    //Gyro
                                    data.gyroX = BUILD_UINT16(19 + i * lengthPerSample,19 + i * lengthPerSample+1);
                                    data.gyroY = BUILD_UINT16(21 + i * lengthPerSample,21 + i * lengthPerSample+1);
                                    data.gyroZ = BUILD_UINT16(23 + i * lengthPerSample,23 + i * lengthPerSample+1);

                                    //Magneto
                                    data.magX = BUILD_UINT16(25 + i * lengthPerSample,25 + i * lengthPerSample+1);
                                    data.magY = BUILD_UINT16(27 + i * lengthPerSample,27 + i * lengthPerSample+1);
                                    data.magZ = BUILD_UINT16(29 + i * lengthPerSample,29 + i * lengthPerSample+1);

                                    printf("IMU OK\n");
                                    IMUData data_=Normalize(data, range, resolutionBits);
                                }
                                    break;
                                case Accel:
                                    //XmlManager.AddGyroData(id, timeStamp, dat1, dat2, dat3, doc);
                                    break;
                                case Gyro:
                                    //XmlManager.AddGyroData(id, timeStamp, dat1, dat2, dat3, doc);
                                    break;
                                case Mag:
                                    //XmlManager.AddMagData(id, timeStamp, dat1, dat2, dat3, doc);
                                    break;
                                case Temperature:
                                    break;
                                case Pressure:
                                    break;
                                case Light:
                                    break;
                                default:
                                    break;
                            }
                        }
                        else
                        {
                            printf("TS IMU Error\n");
                        }
                    }


                }
                break;
            case (short)HS_DATA_PACKET_FULL_TIMESTAMP_V2:
                {
                    SensorType type = (SensorType)payload[0];
                    unsigned char id = payload[1];
                    unsigned char nbChannels = payload[2];
                    float rangeScale = GetFloatSafe(payload,3);
                    unsigned char resolutionBits = payload[7];
                    float samplingFrequency = GetFloatSafe(payload,8);
                    unsigned short nbSamples = payload[12];

                    int lengthPerSample = nbChannels * resolutionBits / 8 + 4;
                    for (int i = 0; i < nbSamples && payloadLength >= lengthPerSample * i + 13; i++)
                    {
                        timeStamp = BUILD_UINT32(payload[13 + i * lengthPerSample+3],payload[13 + i * lengthPerSample+2],payload[13 + i * lengthPerSample+1],payload[13 + i * lengthPerSample]);

                        switch (type)
                        {
                            case IMU:

                                break;
                            case Accel:
                                {
                                    if (lastAccelTimeStamp >= 500000000)
                                        lastAccelTimeStamp = 0;
                                    if (timeStamp > lastAccelTimeStamp)
                                    {
                                        lastAccelTimeStamp = timeStamp;
                                        RAWXYZData dataXYZ;
                                        dataXYZ.timeStamp = timeStamp;
                                        unsigned char datasize = (resolutionBits / 8);
                                        if(datasize==2)
                                        {
                                            dataXYZ.X=BUILD_INT16(payload[17 + i * lengthPerSample],payload[17 + i * lengthPerSample+1]);
                                            dataXYZ.Y=BUILD_INT16(payload[17 + datasize+ i * lengthPerSample],payload[17 +datasize+ i * lengthPerSample+1]);
                                            dataXYZ.Z=BUILD_INT16(payload[17 + 2*datasize+ i * lengthPerSample],payload[17 +2*datasize+ i * lengthPerSample+1]);
                                        }
                                        SensorXYZData datas=NormalizeSensorsDatas(dataXYZ, rangeScale, resolutionBits);
                                        fprintf(sensorsFile, "ACCEL, %d, %f,%f,%f\n", timeStamp, datas.X,datas.Y,datas.Z);
                                    }
                                    else
                                    {
                                        //printf("TS Accel Error\n");
                                    }
                                }
                                break;
                            case Gyro:
                                {
                                    if (lastGyroTimeStamp >= 500000000)
                                        lastGyroTimeStamp = 0;
                                    if (timeStamp > lastGyroTimeStamp)
                                    {
                                        lastGyroTimeStamp = timeStamp;
                                        RAWXYZData dataXYZ;
                                        dataXYZ.timeStamp = timeStamp;
                                        unsigned char datasize = (resolutionBits / 8);
                                        if(datasize==2)
                                        {
                                            dataXYZ.X=BUILD_INT16(payload[17 + i * lengthPerSample],payload[17 + i * lengthPerSample+1]);
                                            dataXYZ.Y=BUILD_INT16(payload[17 + datasize+ i * lengthPerSample],payload[17 +datasize+ i * lengthPerSample+1]);
                                            dataXYZ.Z=BUILD_INT16(payload[17 + 2*datasize+ i * lengthPerSample],payload[17 +2*datasize+ i * lengthPerSample+1]);
                                        }
                                        SensorXYZData datas=NormalizeSensorsDatas(dataXYZ, rangeScale, resolutionBits);
                                        fprintf(sensorsFile, "GYRO, %d, %f,%f,%f\n", timeStamp, datas.X,datas.Y,datas.Z);
                                    }
                                    else
                                    {
                                        //printf("TS Gyro Error\n");
                                    }
                                }

                                break;
                            case Mag:
                                {
                                    if (lastMagTimeStamp >= 500000000)
                                        lastMagTimeStamp = 0;
                                    if (timeStamp > lastMagTimeStamp)
                                    {
                                        lastMagTimeStamp = timeStamp;
                                        RAWXYZData dataXYZ;
                                        dataXYZ.timeStamp = timeStamp;
                                        unsigned char datasize = (resolutionBits / 8);
                                        if(datasize==2)
                                        {
                                            dataXYZ.X=BUILD_INT16(payload[17 + i * lengthPerSample],payload[17 + i * lengthPerSample+1]);
                                            dataXYZ.Y=BUILD_INT16(payload[17 + datasize+ i * lengthPerSample],payload[17 +datasize+ i * lengthPerSample+1]);
                                            dataXYZ.Z=BUILD_INT16(payload[17 + 2*datasize+ i * lengthPerSample],payload[17 +2*datasize+ i * lengthPerSample+1]);
                                        }
                                        SensorXYZData datas=NormalizeSensorsDatas(dataXYZ, rangeScale, resolutionBits);
                                        fprintf(sensorsFile, "MAG, %d, %f,%f,%f\n", timeStamp, datas.X,datas.Y,datas.Z);
                                    }
                                    else
                                    {
                                        //printf("TS Mag Error\n");
                                    }
                                }
                                break;
                            case Temperature:
                                if (lastTemperatureTimeStamp >= 500000000)
                                    lastTemperatureTimeStamp = 0;
                                if (timeStamp > lastTemperatureTimeStamp)
                                {
                                    lastTemperatureTimeStamp = timeStamp;
                                    TemperatureData dataTemperature;
                                    dataTemperature.timeStamp = (double)timeStamp;
                                    unsigned char datasize = (resolutionBits / 8);
                                    dataTemperature.temperature = GetFloatSafe(payload,17 + i * lengthPerSample);
                                    //OnTemperatureDataFromQHB(dataTemperature);
                                    fprintf(sensorsFile, "TEMP, %ld, %lf\n", (unsigned long)dataTemperature.timeStamp, dataTemperature.temperature);

                                }
                                else
                                {
                                    //printf("TS Temperature Error\n");
                                }
                                break;
                            case Pressure:
                                if (lastPressureTimeStamp >= 500000000)
                                    lastPressureTimeStamp = 0;
                                if (timeStamp > lastPressureTimeStamp)
                                {
                                    lastPressureTimeStamp = timeStamp;
                                    PressureData dataPressure;
                                    dataPressure.timeStamp = (double)timeStamp;
                                    unsigned char datasize = (resolutionBits / 8);
                                    dataPressure.pressure = GetFloatSafe(payload,17 + i * lengthPerSample);
                                    //OnPressureDataFromQHB(dataPressure);
                                    fprintf(sensorsFile, "PRESSURE, %ld, %lf\n", (unsigned long)dataPressure.timeStamp, dataPressure.pressure);
                                }
                                else
                                {

                                }
                                break;
                            case Light:
                                if (lastLightTimeStamp >= 500000000)
                                    lastLightTimeStamp = 0;
                                if (timeStamp > lastLightTimeStamp)
                                {
                                    lastLightTimeStamp = timeStamp;
                                    LightData dataLight;
                                    dataLight.timeStamp = timeStamp;
                                    unsigned char datasize = (resolutionBits / 8);
                                    dataLight.ch0 = BUILD_UINT16(payload[17 + i * lengthPerSample],payload[17 + i * lengthPerSample+1]);
                                    dataLight.ch1 = BUILD_UINT16(payload[17 + datasize+i * lengthPerSample],payload[17 +datasize+ i * lengthPerSample+1]);
                                    //OnLightDataFromQHB(dataLight);
                                    fprintf(sensorsFile, "LIGHT, %ld, %d,%d\n", (unsigned long)dataLight.timeStamp, dataLight.ch0,dataLight.ch1);
                                }
                                else
                                {
                                    //printf("TS Light Error\n");
                                }

                                break;
                            
                            default:
                                break;
                        }
                    }
                }
                break;
            case (short)GPS_DATA_PACKET:
                {
                        GPSDatas gpsDatas;
                        unsigned short ms =  BUILD_UINT16(payload[3],payload[4]);
                        if (ms > 999)
                            ms = 0;
                        gpsDatas.dateOfFix.year = payload[7];
                        gpsDatas.dateOfFix.month= payload[6];
                        gpsDatas.dateOfFix.day= payload[5];
                        gpsDatas.dateOfFix.hour= payload[0];
                        gpsDatas.dateOfFix.minute=payload[1];
                        gpsDatas.dateOfFix.second= payload[2];

                        gpsDatas.fix = payload[8];
                        gpsDatas.fixQuality = payload[9];
                        gpsDatas.latitude = GetFloatSafe(payload, 10);
                        gpsDatas.latitudeDirection = (char)payload[14];
                        gpsDatas.longitude = GetFloatSafe(payload, 15);
                        gpsDatas.longitudeDirection = (char)payload[19];
                        gpsDatas.speed = GetFloatSafe(payload, 20);
                        gpsDatas.angle = GetFloatSafe(payload, 24);
                        gpsDatas.altitude = GetFloatSafe(payload, 28);
                        gpsDatas.satellites = payload[32];
                        gpsDatas.antenna = payload[33];

                        if (lastGPSDate.year != gpsDatas.dateOfFix.year ||lastGPSDate.month != gpsDatas.dateOfFix.month ||lastGPSDate.day != gpsDatas.dateOfFix.day || 
                            lastGPSDate.hour!=gpsDatas.dateOfFix.hour || lastGPSDate.minute!=gpsDatas.dateOfFix.minute || lastGPSDate.second!=gpsDatas.dateOfFix.second)
                        {
                            lastGPSDate = gpsDatas.dateOfFix;
                            //OnGPSDataFromQHB(gpsDatas);
                            fprintf(sensorsFile, "GPS, %d/%d/%d %d:%d:%d ", gpsDatas.dateOfFix.year,gpsDatas.dateOfFix.month, gpsDatas.dateOfFix.day,
                                                                            gpsDatas.dateOfFix.hour,gpsDatas.dateOfFix.minute,gpsDatas.dateOfFix.second);
                            fprintf(sensorsFile, "fix:%d, fixQual:%d, Lat:%f %c, lon: %f %c,", gpsDatas.fix,gpsDatas.fixQuality, gpsDatas.latitude, gpsDatas.latitudeDirection,
                                                                            gpsDatas.longitude,gpsDatas.longitudeDirection);
                            fprintf(sensorsFile, "speed:%f, ang:%f, alt:%f, sat:%d\n", gpsDatas.speed,gpsDatas.angle, gpsDatas.altitude, gpsDatas.satellites);                                                                            
                        }
                }
                break;
            case (short)GPS_PPS_PACKET:
                {
                    double PPSTimeStamp =(double) BUILD_UINT64(payload[7],payload[6],payload[5],payload[4],payload[3],payload[2],payload[1],payload[0]);
                    PPSTimeStamp *= 10;     //Pour avoir une unité en nano-seconde (Freq Horloge interne pic32 = 100MHz)

                    if (PPSTimeStamp>lastPPSTimeStampNS)
                    {
                        lastPPSTimeStampNS = PPSTimeStamp;
                        //OnGPSPPSFromQHB(PPSTimeStamp);
                        fprintf(sensorsFile, "PPS:%lf\n", PPSTimeStamp);
                    }
                }
                break;
            default: break;
        }

}