/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.util;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.filters.VFilter;
import com.stuypulse.stuylib.util.StopWatch;

public class TranslationMotionProfile implements VFilter {

    // Default number of times to apply filter (helps accuracy)
    private static final int kDefaultSteps = 64;

    // Stopwatch to Track dt
    private StopWatch mTimer;

    // Limits for each of the derivatives
    private Number mVelLimit;

    private Number mAccelLimit;

    // The last output / velocity
    private Vector2D mOutput;

    private Vector2D mVelocity;

    // Number of times to apply filter (helps accuracy)
    private final int mSteps;

    public TranslationMotionProfile(Number velLimit, Number accelLimit, Vector2D startingTranslation, Vector2D startingVelocity, int steps) {
        mTimer = new StopWatch();
        mVelLimit = velLimit;
        mAccelLimit = accelLimit;
        mOutput = startingTranslation;
        mVelocity = startingVelocity;
        mSteps = steps;
    }

    public TranslationMotionProfile(Number velLimit, Number accelLimit, Vector2D startingTranslation, Vector2D startingVelocity) {
        this(velLimit, accelLimit, startingTranslation, startingVelocity, kDefaultSteps);
    }

    public TranslationMotionProfile(Number velLimit, Number accelLimit) {
        this(velLimit, accelLimit, Vector2D.kOrigin, Vector2D.kOrigin, kDefaultSteps);
    }

    public Vector2D get(Vector2D target) {
        double dt = mTimer.reset() / mSteps;
        for (int i = 0; i < mSteps; ++i) {
            // if there is a accel limit, limit the amount the velocity can change
            if (0 < mAccelLimit.doubleValue()) {
                // amount of windup in system (how long it would take to slow down)
                double windup = mVelocity.magnitude() / mAccelLimit.doubleValue();
                // If the windup is too small, just use normal algorithm to limit acceleration
                if (windup < dt) {
                    // Calculate acceleration needed to reach target
                    Vector2D accel = target.sub(mOutput).div(dt).sub(mVelocity);
                    // Try to reach it while abiding by accel limit
                    mVelocity = mVelocity.add(accel.clamp(dt * mAccelLimit.doubleValue()));
                } else {
                    // the position it would end up if it attempted to come to a full stop
                    Vector2D windA = // windup caused by acceleration
                    mVelocity.mul(0.5 * (dt + windup));
                    // where the robot will end up
                    Vector2D future = mOutput.add(windA);
                    // Calculate acceleration needed to come to stop at target throughout windup
                    Vector2D accel = target.sub(future).div(windup);
                    // Try to reach it while abiding by accel limit
                    mVelocity = mVelocity.add(accel.clamp(dt * mAccelLimit.doubleValue()));
                }
            } else {
                // make the velocity the difference between target and current
                mVelocity = target.sub(mOutput).div(dt);
            }
            // if there is an velocity limit, limit the velocity
            if (0 < mVelLimit.doubleValue()) {
                mVelocity = mVelocity.clamp(mVelLimit.doubleValue());
            }
            Vector2D error = target.sub(mOutput);
            Vector2D unitError = error.normalize();
            double parallelMag = mVelocity.dot(unitError);
            Vector2D accelParallel = unitError.mul(parallelMag);
            Vector2D accelPerpendicular = mVelocity.sub(accelParallel);
            double damping = Math.pow(0.5, dt);
            mVelocity = accelParallel.add(accelPerpendicular.mul(damping));
            // adjust output by calculated velocity
            mOutput = mOutput.add(mVelocity.mul(dt));
        }
        // Field.FIELD2D.getObject("Translation Motion Profile").setPose(!Robot.isBlue()
        // ? new Pose2d(mOutput.x, mOutput.y, new Rotation2d())
        // : Field.transformToOppositeAlliance(new Pose2d(mOutput.x, mOutput.y, new Rotation2d())));
        return mOutput;
    }
}
