package com.stuypulse.robot.util.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public interface Interpolation {
    private static InterpolatingDoubleTreeMap build(double[][] data) {
        InterpolatingDoubleTreeMap doubleMap = new InterpolatingDoubleTreeMap();
        for (double[] d : data) doubleMap.put(d[0], d[1]);
        return doubleMap;
    }

    public class Shooting {
        private static final double[][] DistanceAndRPM = {
            {0, 50},
            {5, 100},
            {10, 200}
        };

        private static final InterpolatingDoubleTreeMap interpolator = build(DistanceAndRPM);

        public static double getRPM(double distance) {
            return interpolator.get(distance);
        }

        public static InterpolatingDoubleTreeMap getInterpolator() {
            return interpolator;
        }
    }

    public class Ferrying {
        private static final double[][] RPMAndDistance = {
            {5, 100}
        };

        private static final InterpolatingDoubleTreeMap interpolator = build(RPMAndDistance);

        public static double getRPM(double distance) {
            return interpolator.get(distance);
        }
        
        public static InterpolatingDoubleTreeMap getInterpolator() {
            return interpolator;
        }
    }

    public class TOF { // time of flight
        private static final double[][] RPMAndDistance = {
            {6, 7}
        };

        private static final InterpolatingDoubleTreeMap interpolator = build(RPMAndDistance);

        public static double getRPM(double distance) {
            return interpolator.get(distance);
        }

        public static InterpolatingDoubleTreeMap getInterpolator() {
            return interpolator;
        }
    }
}