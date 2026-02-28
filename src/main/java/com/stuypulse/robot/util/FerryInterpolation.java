package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class FerryInterpolation {
    private static final InterpolatingDoubleTreeMap interpolater;


    //TODO: Get from testing
    private static final double[][] RPMAndDistance = {
        {5,100},
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