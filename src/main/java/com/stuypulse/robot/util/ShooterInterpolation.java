package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolation {
    private static final InterpolatingDoubleTreeMap interpolater;


    //TODO: Get from testing
    private static final double[][] RPMAndDistance = {
        {0, 1000},
        {5, 2000},
        {10, 3000}
    };

    static {
        interpolater = new InterpolatingDoubleTreeMap();
        for (double[] data:RPMAndDistance) {
            interpolater.put(data[0], data[1]);
        }
    }

    public static double getRPM(double distance) {
        return interpolater.get(distance);
    }
}