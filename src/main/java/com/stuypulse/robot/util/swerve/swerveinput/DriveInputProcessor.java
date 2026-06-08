/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.swerve.swerveinput;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

/**
 * <h2>DriveInputProcessor</h2>
 * <p>Class for processing driver input for the drivetrain. It's intended to replace StuyLib's VStream 
 * and Filters and use pure WPILib and Java.</p>
 * <p>This will be used to be as close as the original StuyLib implementation as possible</p>
 * 
 * It will be used for
 * <ul>
 *   <li>Applying deadbands</li>
 *   <li>Applying power curves</li>
 *   <li>Applying rate limits</li>
 *   <li>Applying a low pass filter</li>
 * </ul>
 * 
 * <p>To use within a command, create an instance of this class and call the {@link #update()} method in the {@link edu.wpi.first.wpilibj2.command.Command#execute()} method of the command.</p>
 * <p>To get the processed speed, call the {@link #get()} method.</p>
 */
public class DriveInputProcessor {
    private final CommandXboxController controller;
    private final double deadband;
    private final double clamp;
    private final double power;
    /** The max velocity in meters per second/ */
    private final double maxVelocity;

    private final SlewRateLimiter xRateLimiter;
    private final SlewRateLimiter yRateLimiter;

    private final LinearFilter xLowPassFilter;
    private final LinearFilter yLowPassFilter;

    /**
     * The speed vector after the full processing and filtering.
     */
    private Translation2d processedSpeed;

    /**
     * <h4>Constructor for the DriveInputProcessor</h4>
     * <p>Creates a new DriveInputProcessor with the specified parameters and processes the input periodically.</p>
     * @param controller The CommandXboxController to get driver input from
     * @param deadband The deadband to apply to the input (0-1)
     * @param clamp The maximum magnitude of the input vector (0-1)
     * @param power The power to apply to the input
     * @param maxVelocity The maximum velocity to scale the input to (m/s)
     * @param maxAccel The maximum acceleration to apply to the input (m/s^2)
     * @param rc The time constant for the low pass filter (seconds)
     */
    public DriveInputProcessor(CommandXboxController controller, double deadband, double clamp, double power, double maxVelocity, double maxAcceleration, double rc) {
        this.controller = controller;
        this.deadband = deadband;
        this.clamp = clamp;
        this.power = power;
        this.maxVelocity = maxVelocity;

        this.xRateLimiter = new SlewRateLimiter(maxAcceleration);
        this.yRateLimiter = new SlewRateLimiter(maxAcceleration);
        this.xLowPassFilter = LinearFilter.singlePoleIIR(rc, Settings.DT.in(Seconds));
        this.yLowPassFilter = LinearFilter.singlePoleIIR(rc, Settings.DT.in(Seconds));

        this.processedSpeed = Translation2d.kZero;
    }

    /**
     * Read the raw joystick axes and store them as a velocity vector in {@link #processedSpeed}
     *
     * @return This instance of the class
     */
    private DriveInputProcessor getDriverInputAsVelocity() {
        this.processedSpeed = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
        return this;
    }

    /**
     * Apply a deadband on the X and Y axes to the current {@link #processedSpeed} vector
     * using {@link MathUtil#applyDeadband}
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyDeadband() {
        double deadbandX = MathUtil.applyDeadband(this.processedSpeed.getX(), deadband);
        double deadbandY = MathUtil.applyDeadband(this.processedSpeed.getY(), deadband);
        this.processedSpeed = new Translation2d(deadbandX, deadbandY);
        return this;
    }

    /** 
     * Limits the magnitude of the current {@link #processedSpeed} vector to a maximum
     * 
     * @return This instance of the class
    */
    private DriveInputProcessor applyClamp() {
        double magnitude = this.processedSpeed.getNorm();

        if (magnitude > clamp) {
            this.processedSpeed = this.processedSpeed.times(clamp / magnitude);
        }

        return this;
    }

    /**
     * Apply a power curve to the current {@link #processedSpeed} vector.
     * The vector's magnitude is raised according to the configured power
     * while preserving direction.
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyPowerCurve() {
        this.processedSpeed = this.processedSpeed.times(Math.pow(this.processedSpeed.getNorm(), power - 1));
        return this;
    }

    /**
     * Scale the {@link #processedSpeed} vector to the robot's maximum velocity.
     * Multiplies the current vector by maxVelocity (m/s).
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyScalingToMaxVelocity() {
        this.processedSpeed = this.processedSpeed.times(maxVelocity);
        return this;
    }

    /**
     * Applies a rate limit to {@link #processedSpeed} using 
     * a {@link edu.wpi.first.math.filter.SlewRateLimiter} for each axis. 
     * This limits the rate of change of the velocity vector to the 
     * max acceleration (m/s^2).
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyRateLimit() {
        double limitedX = xRateLimiter.calculate(this.processedSpeed.getX());
        double limitedY = yRateLimiter.calculate(this.processedSpeed.getY());
        this.processedSpeed = new Translation2d(limitedX, limitedY);
        return this;
    }
    
    /**
     * Applies a low pass filter to {@link #processedSpeed} using
     * a {@link edu.wpi.first.math.filter.LinearFilter} for each axis. 
     * This smooths out the velocity vector and reduces noise.
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyLowPassFilter() {
        double filteredX = xLowPassFilter.calculate(this.processedSpeed.getX());
        double filteredY = yLowPassFilter.calculate(this.processedSpeed.getY());
        this.processedSpeed = new Translation2d(filteredX, filteredY);
        return this;
    }
    

    /**
     * Update method that processes all filters and updates the filtered speed.
     * This should be called within the start of the {@link edu.wpi.first.wpilibj2.command.Command#execute()} 
     * method of the command using DriveInputProcessor.
     */
    public void update() {
        getDriverInputAsVelocity()
            .applyDeadband()
            .applyPowerCurve()
            .applyClamp()
            .applyScalingToMaxVelocity()
            .applyRateLimit()
            .applyLowPassFilter();
    }

    /**
     * Get the processed speed.
     * @return A Translation2d representing the processed speed
     */
    public Translation2d get() {
        return processedSpeed;
    }
}
