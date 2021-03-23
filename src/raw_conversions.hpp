#ifndef RAW_CONVERSIONS_HPP
#define RAW_CONVERSIONS_HPP

#include <stdint.h>
#include <cmath>
#include <limits.h>

#define RAD2DEG 57.295779513

float convertAPPS(uint16_t apps1Raw, uint16_t apps2Raw)
{
    //APPS1 ADC to Degrees
    double u = (double)(4095 - apps1Raw);
    double stroke = (-4e-11*u*u*u + 3e-8*u*u + 0.0066*u + 2.1254);
    stroke += 143.40 - 20.35;
    double a = 68.59, b = 120.28;
    double angle1 = acos(-(a*a + b*b- stroke*stroke)/(2*a*b)) * RAD2DEG;
    //APPS2 ADC to Degrees
    u = (double)(4095 - apps2Raw);
    stroke = (-5e-11*u*u*u + 1e-7*u*u + 0.0064*u + 1.9078);
    stroke += 123;
    a = 61.39, b = 120.28;
    double angle2 = acos(-(a*a + b*b- stroke*stroke)/(2*a*b)) * RAD2DEG;

    double angleMean = (angle1 + angle2) * 0.5;

    float apps = ((angleMean - 81.5) / (94.5 - 81.5));
    apps    = apps < 0.0 ? 0.0  //Saturate between [0,1]
            : apps > 1.0 ? 1.0
            : apps;

    return apps;
}

float convertBrakePressure(uint16_t brakeRaw)
{
    float brake = (double)(brakeRaw*5)/4095.0; //ADCtoVolt
    brake -= 0.5;                               //OffsetRemoval
    brake = brake * 2500.0 / 4.0;               //VoltToPsi
    brake = brake / 14.5038;                    //PsiToBar
    brake = brake < 0.0 ? 0.0 : brake/60.0;     //Saturate and scale
    return brake;
}

float convertEbsPressure(uint16_t ebsRaw)
{
    double pressure = 2.5 * ((float)ebsRaw) - 2.5; //linear conversion
    return pressure;                                //1V -> 0bar | 5V -> 10bar
}

float convertFrontRPM(uint16_t hall)
{
    return (float)(((1/30.0)/((float)(hall)*0.000001))*60.0);
}

uint16_t convertRearRPM(uint16_t hall)
{
    return (uint16_t)(((1/35.0)/((float)(hall)*0.000001))*60.0);
}

float convertSteeringActual(uint16_t steeringRaw)
{
    return (float)((-2.0*(float)steeringRaw/2353.0)+(3993.3/2353.0));
}

/*********************From APU to other devices*********************/

uint16_t convertSteeringAngleTarget(float targetSteeringAngle)
{
    return (uint16_t)((-2353.0*targetSteeringAngle/2.0)+(3993.3/2.0));
}

uint32_t convertSteeringRateTarget(float targetSteeringRate)
{
    uint32_t steeringRateRaw = targetSteeringRate < 0.01 ? 0 : (uint32_t)(941.0/fabs(targetSteeringRate));
    return steeringRateRaw;
}

int16_t convertThrottleTarget(float targetThrottle) 
{
    return (uint16_t)(targetThrottle*SHRT_MAX);
}

#endif
