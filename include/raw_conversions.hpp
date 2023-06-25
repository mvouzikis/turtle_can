#ifndef RAW_CONVERSIONS_HPP
#define RAW_CONVERSIONS_HPP

#include <stdint.h>
#include <cmath>
#include <limits.h>


float convertAPPS(uint16_t apps1Raw, uint16_t apps2Raw)
{
    //APPS1 ADC to Degrees
    double u = (double)(4095 - apps1Raw);
    double stroke = (-4e-11*u*u*u + 3e-8*u*u + 0.0066*u + 2.1254);
    stroke += 143.40 - 20.35;
    double a = 68.59, b = 120.28;
    double angle1 = acos(-(a*a + b*b- stroke*stroke)/(2*a*b)) * 180 / M_PI;
    
    //APPS2 ADC to Degrees
    u = (double)(4095 - apps2Raw);
    stroke = (-5e-11*u*u*u + 1e-7*u*u + 0.0064*u + 1.9078);
    stroke += 123;
    a = 61.39, b = 120.28;
    double angle2 = acos(-(a*a + b*b- stroke*stroke)/(2*a*b)) * 180 / M_PI;

    double angleMean = (angle1 + angle2) * 0.5;

    float apps = ((angleMean - 81.5) / (94.5 - 81.5));
    apps    = apps < 0.0 ? 0.0  //Saturate between [0,1]
            : apps > 1.0 ? 1.0
            : apps;

    return apps;
}

float convertFrontRPM(uint16_t hall)
{
    return (float)(((1/30.0)/((float)(hall)*0.000001))*60.0);
}

uint16_t convertRearRPM(uint16_t hall)
{
    return (uint16_t)(((1/35.0)/((float)(hall)*0.000001))*60.0);
}

float convertSteeringActual(float steering_raw, int16_t steering_offset)
{
    float steering_ratio = 5.0;
    float steering_offset_from_ecu = steering_offset / 100.0;  // the scaling factor is defined in the ecu code

    float steeringActual = (steering_raw + steering_offset_from_ecu) / steering_ratio; // convert degrees from rack to vehicle's wheels
    
    return steeringActual;
}

float convertBLDCSteering(float BLDC_steering)
{
    float BLDC_gear_box_ratio = 25.71;
    float steering_ratio = 5.0;

    float BLDC_actual_steering = BLDC_steering / 1000.0; // convert from thousandth of degrees to degrees
    BLDC_actual_steering = -BLDC_actual_steering; // invert it
    BLDC_actual_steering = BLDC_actual_steering / BLDC_gear_box_ratio;  // convert degrees from BLDC to steering wheel
    BLDC_actual_steering = BLDC_actual_steering / steering_ratio; // convert degrees from steering wheel to wheel
    BLDC_actual_steering = BLDC_actual_steering * M_PI / 180; // convert value from degrees to rads

    return BLDC_actual_steering;
}

/*********************From APU to other devices*********************/

uint16_t convertSteeringAngleTarget(float targetSteeringAngle)
{
    return (uint16_t)((-1073.0*targetSteeringAngle)+(2014.0));
}

uint32_t convertSteeringRateTarget(float targetSteeringRate)
{
    uint32_t steeringRateRaw = fabs(targetSteeringRate) < 0.01 ? 0 : (uint32_t)(941.0/fabs(targetSteeringRate));
    return steeringRateRaw;
}

uint16_t convertCPUTemp(float CPUTemp[], uint16_t CoresNum)
{
    return (uint16_t)(*std::max_element(CPUTemp, CPUTemp + CoresNum));
}

#endif
