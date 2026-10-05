package com.stuypulse.robot.constants;

public interface InterpolationConstants {
    double[][] DISTANCE_RPM_INTERPOLATION_VALUES = {
        {1.46, 2600},
        {2.07, 3150},
        {3.13, 3700},
        {3.45, 3933},
        {4.13, 4200}
        //TODO: These numbers don't make sense
        // { 4.895367348608047, 3250.0 },
        // { 6.1322461808798705, 3487.0 } 
    };

    double[][] DISTANCE_TOF_INTERPOLATION_VALUES = {
        { 1.0, 0.5 },
        { 2.0, 0.75 },
        { 3.0, 1.0 },
        { 4.0, 1.25 },
        { 5.0, 1.5 } 
    };

    double[][] FERRY_DISTANCE_RPM_INTERPOLATION_VALUES = {
        { 1.0, 2300.0 },
        { 2.0, 2800.0 },
        { 3.0, 3300.0 },
        { 4.0, 3800.0 },
        { 5.0, 5500.0 } 
    };

    double[][] FERRY_TOF_INTERPOLATION_VALUES = {
        { 1.0, 0.5 },
        { 2.0, 0.75 },
        { 3.0, 1.0 },
        { 4.0, 1.25 },
        { 5.0, 1.5 } 
    };
}
