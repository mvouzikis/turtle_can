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
    float pressure = 2.5 * ((5.0/4095.0)*((float)ebsRaw)) - 2.5; //linear conversion
    return pressure;                                           //1V -> 0bar | 5V -> 10bar
}

float convertFrontRPM(uint16_t hall)
{
    return (float)(((1/30.0)/((float)(hall)*0.000001))*60.0);
}

uint16_t convertRearRPM(uint16_t hall)
{
    return (uint16_t)(((1/35.0)/((float)(hall)*0.000001))*60.0);
}

float convertSteeringActual(float steeringRaw, int16_t steering_offset)
{
    float RackToVehicleWheelRatio = 6.95238;
    float steering_offset_from_ecu = steering_offset / 100.0;  // the scaling factor is defined in the ecu code

    float steeringActual = steeringRaw / RackToVehicleWheelRatio; // convert degrees from rack to vehicle's wheels
    
    return steeringActual + steering_offset_from_ecu;
}

float convertBLDCSteering(float BLDCSteering)
{
    float BLDCToSteeringWheelGR = 3.68;
    float PinionToRackRatio = 7.0;
    float RackToVehicleWheelRatio = 6.95238;

    float BLDCActualSteering = BLDCSteering / 1000.0; // convert from thousandth of degrees to degrees
    BLDCActualSteering = -BLDCActualSteering; // invert it
    BLDCActualSteering = BLDCActualSteering / BLDCToSteeringWheelGR;  // convert degrees from BLDC to steering wheel
    BLDCActualSteering = BLDCActualSteering / PinionToRackRatio; // convert degrees from steering wheel to rack
    BLDCActualSteering = BLDCActualSteering / RackToVehicleWheelRatio; // convert degrees from rack to vehicle's wheels
    BLDCActualSteering = BLDCActualSteering / RAD2DEG; // convert value from degrees to rads

    return BLDCActualSteering;
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
