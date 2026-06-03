/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class TranslationMotionProfile {

    // Default number of times to apply filter (helps accuracy)
    private static final int kDefaultSteps = 64;

    // Limits for each of the derivatives
    private final Number velocityLimit;

    private final Number accelerationLimit;

    // The last output / velocity
    private Translation2d output;

    private Translation2d velocity;

    // Number of times to apply filter (helps accuracy)
    private final int steps;

    private double previousTime;

    public TranslationMotionProfile(
            Number velLimit,
            Number accelLimit,
            Translation2d startingTranslation,
            Translation2d startingVelocity,
            int steps) {
        velocityLimit = velLimit;
        accelerationLimit = accelLimit;
        output = startingTranslation;
        velocity = startingVelocity;
        this.steps = steps;
    }

    public TranslationMotionProfile(
            Number velLimit, Number accelLimit, Translation2d startingTranslation, Translation2d startingVelocity) {
        this(velLimit, accelLimit, startingTranslation, startingVelocity, kDefaultSteps);
    }

    public TranslationMotionProfile(Number velLimit, Number accelLimit) {
        this(velLimit, accelLimit, Translation2d.kZero, Translation2d.kZero, kDefaultSteps);
    }

    private static Translation2d clamp(Translation2d translation, double maxMagnitude) {
        if (maxMagnitude <= 0.0) return Translation2d.kZero;

        final double magnitude = translation.getNorm();

        if (maxMagnitude <= magnitude) {
            return translation.times(maxMagnitude / magnitude);
        }

        return translation;
    }

    public static Translation2d normalize(Translation2d translation) {
        final double magnitude = translation.getNorm();
        if (magnitude <= 1e-9) {
            return new Translation2d(1, 0);
        }
        return translation.div(magnitude);
    }

    public Translation2d get(Translation2d target) {
        double dt = resetTimer() / steps;
        for (int i = 0; i < steps; ++i) {
            // if there is a accel limit, limit the amount the velocity can change
            if (0 < accelerationLimit.doubleValue()) {
                // amount of windup in system (how long it would take to slow down)
                double windup = velocity.getNorm() / accelerationLimit.doubleValue();
                // If the windup is too small, just use normal algorithm to limit acceleration
                if (windup < dt) {
                    // Calculate acceleration needed to reach target
                    Translation2d accel = target.minus(output).div(dt).minus(velocity);
                    // Try to reach it while abiding by accel limit
                    velocity = velocity.plus(clamp(accel, dt * accelerationLimit.doubleValue()));
                } else {
                    // the position it would end up if it attempted to come to a full stop
                    Translation2d windA = // windup caused by acceleration
                            velocity.times(0.5 * (dt + windup));
                    // where the robot will end up
                    Translation2d future = output.plus(windA);
                    // Calculate acceleration needed to come to stop at target throughout windup
                    Translation2d accel = target.minus(future).div(windup);
                    // Try to reach it while abiding by accel limit
                    velocity = velocity.plus(clamp(accel, dt * accelerationLimit.doubleValue()));
                }
            } else {
                // make the velocity the difference between target and current
                velocity = target.minus(output).div(dt);
            }
            // if there is an velocity limit, limit the velocity
            if (0 < velocityLimit.doubleValue()) {
                velocity = clamp(velocity, velocityLimit.doubleValue());
            }
            Translation2d error = target.minus(output);
            Translation2d unitError = normalize(error);
            double parallelMag = velocity.dot(unitError);
            Translation2d accelParallel = unitError.times(parallelMag);
            Translation2d accelPerpendicular = velocity.minus(accelParallel);
            double damping = Math.pow(0.5, dt);
            velocity = accelParallel.plus(accelPerpendicular.times(damping));
            // adjust output by calculated velocity
            output = output.plus(velocity.times(dt));
        }
        return output;
    }

    private double resetTimer() {
        double now = Timer.getFPGATimestamp();
        double elapsed = now - previousTime;
        previousTime = now;
        return elapsed;
    }
}