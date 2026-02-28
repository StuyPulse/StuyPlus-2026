package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolation {
    private static final InterpolatingDoubleTreeMap interpolater;


    //TODO: Get from testing
    private static final double[][] RPMAndDistance = {
        {0,0},
    };

    static {
        interpolater = new InterpolatingDoubleTreeMap();
        for (double[] data:RPMAndDistance) {
            interpolater.put(data[1], data[0]);
        }
    }

    public static double getRPM(double distance) {
        return interpolater.get(distance);
    }
}