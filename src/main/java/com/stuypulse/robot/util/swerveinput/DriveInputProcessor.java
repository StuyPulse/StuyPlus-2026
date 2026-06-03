package com.stuypulse.robot.util.swerveinput;

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
    private CommandXboxController controller;
    private double deadband;
    private double power;
    private double maxVelocity; // m/s
    private double maxAcceleration; // m/s^2
    private double rc; // low pass filter time constant

    private SlewRateLimiter xRateLimiter;
    private SlewRateLimiter yRateLimiter;

    private LinearFilter xLowPassFilter;
    private LinearFilter yLowPassFilter;

    /**
     * The speed vector during processing, before filtering.
     */
    private Translation2d intermediateProcessedSpeed;

    /**
     * The speed vector after the full processing and filtering.
     */
    private Translation2d filteredSpeed;

    /**
     * <h4>Constructor for the DriveInputProcessor</h4>
     * <p>Creates a new DriveInputProcessor with the specified parameters and creates a notifier to run periodically (20ms) and process the input.</p>
     * @param controller The CommandXboxController to get driver input from
     * @param deadband The deadband to apply to the input (0-1)
     * @param power The power to apply to the input
     * @param maxVelocity The maximum velocity to scale the input to (m/s)
     * @param maxAccel The maximum acceleration to apply to the input (m/s^2)
     * @param rc The time constant for the low pass filter (seconds)
     */
    public DriveInputProcessor(CommandXboxController controller, double deadband, double power, double maxVelocity, double maxAccel, double rc) {
        this.controller = controller;
        this.deadband = deadband;
        this.power = power;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAccel;
        this.rc = rc;

        this.xRateLimiter = new SlewRateLimiter(maxAcceleration);
        this.yRateLimiter = new SlewRateLimiter(maxAcceleration);
        this.xLowPassFilter = LinearFilter.singlePoleIIR(this.rc, Settings.DT.in(Seconds));
        this.yLowPassFilter = LinearFilter.singlePoleIIR(this.rc, Settings.DT.in(Seconds));

        this.intermediateProcessedSpeed = Translation2d.kZero;
        this.filteredSpeed = Translation2d.kZero;
    }

    /**
     * Read the raw joystick axes and store them as a velocity vector in {@link #filteredSpeed}
     *
     * @return This instance of the class
     */
    private DriveInputProcessor getDriverInputAsVelocity() {
            this.intermediateProcessedSpeed = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
            return this;
    }

    /**
     * Apply a deadband on the X and Y axes to the current {@link #filteredSpeed}
     * using {@link MathUtil#applyDeadband}
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyDeadband() {
        double deadbandX = MathUtil.applyDeadband(this.intermediateProcessedSpeed.getX(), deadband);
        double deadbandY = MathUtil.applyDeadband(this.intermediateProcessedSpeed.getY(), deadband);
        this.intermediateProcessedSpeed = new Translation2d(deadbandX, deadbandY);
        return this;
    }

    /**
     * Apply a power curve to the current {@link #intermediateProcessedSpeed} vector.
     * The vector's magnitude is raised according to the configured power
     * while preserving direction.
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyPowerCurve() {
        this.intermediateProcessedSpeed = this.intermediateProcessedSpeed.times(Math.pow(this.intermediateProcessedSpeed.getNorm(), power - 1));
        return this;
    }

    /**
     * Scale the {@link #intermediateProcessedSpeed} vector to the robot's maximum velocity.
     * Multiplies the current vector by maxVelocity (m/s).
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyScalingToMaxVelocity() {
        this.intermediateProcessedSpeed = this.intermediateProcessedSpeed.times(maxVelocity);
        return this;
    }

    /**
     * Applies a rate limit to {@link #intermediateProcessedSpeed} using 
     * a {@link edu.wpi.first.math.filter.SlewRateLimiter} for each axis. 
     * This limits the rate of change of the velocity vector to the 
     * {@link #maxAcceleration} (m/s^2).
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyRateLimit() {
        double limitedX = xRateLimiter.calculate(this.intermediateProcessedSpeed.getX());
        double limitedY = yRateLimiter.calculate(this.intermediateProcessedSpeed.getY());
        this.intermediateProcessedSpeed = new Translation2d(limitedX, limitedY);
        return this;
    }
    
    /**
     * Applies a low pass filter to {@link #intermediateProcessedSpeed} using
     * a {@link edu.wpi.first.math.filter.LinearFilter} for each axis. 
     * This smooths out the velocity vector and reduces noise.
     *
     * @return This instance of the class
     */
    private DriveInputProcessor applyLowPassFilter() {
        double filteredX = xLowPassFilter.calculate(this.intermediateProcessedSpeed.getX());
        double filteredY = yLowPassFilter.calculate(this.intermediateProcessedSpeed.getY());
        this.intermediateProcessedSpeed = new Translation2d(filteredX, filteredY);
        return this;
    }
    

    /**
     * Update method that processes all filters and updates the filtered speed.
     * This should be called within the {@link edu.wpi.first.wpilibj2.command.Command#execute()} 
     * method of the command using DriveInputProcessor.
     */
    public void update() {
        getDriverInputAsVelocity()
            .applyDeadband()
            .applyPowerCurve()
            .applyScalingToMaxVelocity()
            .applyRateLimit()
            .applyLowPassFilter();
        this.filteredSpeed = this.intermediateProcessedSpeed;
    }

    /**
     * Get the filtered speed.
     * @return A Translation2d representing the filtered speed
     */
    public Translation2d get() {
        return filteredSpeed;
    }
}
