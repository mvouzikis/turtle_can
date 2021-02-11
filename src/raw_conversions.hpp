#ifndef RAW_CONVERSIONS_HPP
#define RAW_CONVERSIONS_HPP

#include <stdint.h>
#include <cmath>

#define RAD2DEG 57.295779513

double convertAPPS(uint16_t apps1Raw, uint16_t apps2Raw)
{
    //APPS1 ADC to Degrees
    double u = (double)(4095 - apps1Raw);
    double stroke = (-4e-11*u*u*u + 3e-8*u*u + 0.0066*u + 2.1254);
    double a = 68.59, b = 120.28;
    double angle1 = acos(-(a*a + b*b- stroke*stroke)/(2*a*b)) * RAD2DEG;
    //APPS2 ADC to Degrees
    u = (double)(4095 - apps2Raw);
    stroke = (-5e-11*u*u*u + 1e-7*u*u + 0.0064*u + 1.9078);
    a = 61.39, b = 120.28;
    double angle2 = acos(-(a*a + b*b- stroke*stroke)/(2*a*b)) * RAD2DEG;

    double angleMean = (angle1 + angle2) * 0.5;

    return ((angleMean - 80) / 15);
}

double convertBrakePressure(uint16_t brakeRaw)
{
    double brake = (double)(brakeRaw*5)/4095.0; //ADCtoVolt
    brake -= 0.5;                               //OffsetRemoval
    brake = brake * 2500.0 / 4.0;               //VoltToPsi
    return (brake / 14.5038);                   //PsiToBar
}

uint16_t convertFrontRPM(uint16_t hall)
{
    return (uint16_t)(((1/30.0)/((float)(hall)*0.000001))*60.0);
}

uint16_t convertRearRPM(uint16_t hall)
{
    return (uint16_t)(((1/35.0)/((float)(hall)*0.000001))*60.0);
}

#endif