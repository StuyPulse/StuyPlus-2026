package com.stuypulse.robot.util.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolation {
    private static final InterpolatingDoubleTreeMap interpolater;


    //TODO: Get from testing
    public static final double[][] RPMAndDistance = {
        {0, 50},
        {5, 100},
        {10, 200}
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