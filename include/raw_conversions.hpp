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

float convertSteeringActual(float steering_raw, float steering_offset)
{
    float steering_ratio = 5.0;
    // float steering_offset_from_ecu = steering_offset / 10000.0;  // the scaling factor is defined in the ecu code

    // float steeringActual = (steering_raw + steering_offset_from_ecu) / steering_ratio; // convert degrees from rack to vehicle's wheels
    float steeringActual = steering_raw;
    
    if (steeringActual < -0.2) steeringActual += 2 * M_PI;
    
    steeringActual -= 1.8;
    steeringActual /= steering_ratio;
    return steeringActual;
}

float convertFromBLDCToSteering(float BLDC_steering)
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

float lookup_table(float input, float breakpoints [], float table_data [], int TABLE_SIZE)  {
    float x1, x2, y1, y2;
    // Handle cases where input is out of table range
    if (input <= breakpoints[0]) {
        return table_data[0]; // Clamp to minimum value
    }
    if (input >= breakpoints[TABLE_SIZE - 1]) {
        return table_data[TABLE_SIZE - 1]; // Clamp to maximum value
    }
 
    // Perform linear interpolation
    for (int i = 1; i < TABLE_SIZE - 1; i++) {
        if (input >= breakpoints[i] && input <= breakpoints[i + 1]) {
            // Interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            x1 = breakpoints[i], x2 = breakpoints[i + 1];
            y1 = table_data[i], y2 = table_data[i + 1];
            return y1 + (input - x1) * (y2 - y1) / (x2 - x1);
					}
    }
 
    // Default case (should never reach here)
    return 0.0;
}

float convertToBLDCFromSteering(float BLDC_steering) {
    
    float breakpoints [] = {-0.482077222,-0.463411667,-0.445007778,-0.426691111,-0.408548889,-0.390581111,-0.372700556,-0.354994444,-0.337375556,-0.319931111,-0.302486667,-0.285129444,-0.268033889,-0.250763889,-0.233842778,-0.216834444,-0.199826111,-0.183341111,-0.166245556,-0.149498889,-0.132752222,-0.116092778,-0.099433333,-0.082773889,-0.066288889,-0.049716667,-0.033057222,-0.016572222,0,0.016572222,0.033057222,0.049716667,0.066288889,0.082773889,0.099433333,0.116092778,0.132752222,0.149498889,0.166245556,0.183341111,0.199826111,0.216834444,0.233842778,0.250763889,0.268033889,0.285129444,0.302486667,0.319931111,0.337375556,0.354994444,0.372700556,0.390581111,0.408548889,0.426691111,0.445007778,0.463411667,0.482077222};

    float table_data [] = {-2.33333,-2.25,-2.16667,-2.08333,-2,-1.91667,-1.83333,-1.75,-1.66667,-1.58333,-1.5,-1.41667,-1.33333,-1.25,-1.16667,-1.08333,-1,-0.91667,-0.83333,-0.75,-0.66667,-0.58333,-0.5,-0.41667,-0.33333,-0.25,-0.16667,-0.08333,0,0.083333333,0.166666667,0.25,0.333333333,0.416666667,0.5,0.583333333,0.666666667,0.75,0.833333333,0.916666667,1,1.083333333,1.166666667,1.25,1.333333333,1.416666667,1.5,1.583333333,1.666666667,1.75,1.833333333,1.916666667,2,2.083333333,2.166666667,2.25,2.333333333};
 
    int TABLE_SIZE = 57;

    float steering_angle = 0;

    steering_angle = (lookup_table(BLDC_steering, breakpoints, table_data, TABLE_SIZE)*180.0/M_PI);//steering_angle = (lookup_table()*180.0/PI); debug_command steering_command.position_target
	if (steering_angle>85) steering_angle = 85; // Must be changed alonside the lookup table in the new car possibly
	else if (steering_angle < -85) steering_angle = -85; // Must be changed alonside the lookup table in the new car possibly
	steering_angle = steering_angle * (-25710.0); //BLDC thousands of degrees

    return steering_angle;
}


/*********************From APU to other devices*********************/

uint16_t convertCPUTemp(float CPUTemp[], uint16_t CoresNum)
{
    return (uint16_t)(*std::max_element(CPUTemp, CPUTemp + CoresNum));
}

#endif
