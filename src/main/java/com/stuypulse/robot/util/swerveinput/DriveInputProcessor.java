package com.stuypulse.robot.util.swerveinput;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

/**
 * <h2>DriveInputProcessor</h2>
 * <p>Class for processing driver input for the drivetrain. It's intended to replace StuyLib's VStream 
 * and Filters and use pure WPILIB and Java.</p>
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

    // for rate limiting
    private Translation2d rateLimitLastVelocity;

    // for low pass filter
    private double lowPassLastX;
    private double lowPassLastY;

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

        this.rateLimitLastVelocity = Translation2d.kZero;
        this.lowPassLastX = 0;
        this.lowPassLastY = 0;

        this.filteredSpeed = Translation2d.kZero;
    }

    /**
     * Get the driver input from the controller as a Translation2d, representing as a velocity vector
     * @return Translation2d representing the driver input as a velocity vector
     */
    private Translation2d getDriverInputAsVelocity() {
            return new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    }

    /**
     * Applies a deadband to the given Translation2d input.
     * @param input The input Translation2d to apply the deadband to
     * @return A Translation2d with the deadband applied
     */
    private Translation2d applyDeadband(Translation2d input) {
        double deadbandX = MathUtil.applyDeadband(input.getX(), deadband);
        double deadbandY = MathUtil.applyDeadband(input.getY(), deadband);
        return new Translation2d(deadbandX, deadbandY);
    }

    /**
     * Applies a power curve to the given Translation2d input.
     * @param input The input Translation2d to apply the power curve to
     * @return A Translation2d with the power curve applied
     */
    private Translation2d applyPowerCurve(Translation2d input) {
        return input.times(Math.pow(input.getNorm(), power - 1));
    }

    /**
     * Scales the given Translation2d input to the maximum velocity.
     * @param input The input Translation2d to apply the scaling to
     * @return A Translation2d scaled to the maximum velocity
     */
    private Translation2d scaleToMaxVelocity(Translation2d input) {
        return input.times(maxVelocity);
    }

    /**
     * Applies a rate limit to the given Translation2d input.
     * @param input The input Translation2d to apply the rate limit to
     * @return A Translation2d with the rate limit applied
     */
    private Translation2d applyRateLimit(Translation2d input) {
        double maxDelta = maxAcceleration * Settings.DT.in(Seconds);
        Translation2d delta = input.minus(rateLimitLastVelocity);
        
        double deltaMagnitude = delta.getNorm();
        if (deltaMagnitude > maxDelta && deltaMagnitude > 1e-9) {
            delta = delta.times(maxDelta / deltaMagnitude);
        }

        Translation2d limited = rateLimitLastVelocity.plus(delta);
        rateLimitLastVelocity = limited;
        return limited;
    }

    /**
     * Applies a low pass filter to the given Translation2d input.
     * @param input The input Translation2d to apply the low pass filter to
     * @return A Translation2d with the low pass filter applied
     */
    private Translation2d applyLowPassFilter(Translation2d input) {
        if (rc <= 0) {
            lowPassLastX = input.getX();
            lowPassLastY = input.getY();
            return input;
        }

        double alphaX = 1.0 - Math.exp(-Settings.DT.in(Seconds) / rc);
        double alphaY = 1.0 - Math.exp(-Settings.DT.in(Seconds) / rc);

        double filteredX = lowPassLastX + alphaX * (input.getX() - lowPassLastX);
        double filteredY = lowPassLastY + alphaY * (input.getY() - lowPassLastY);
        lowPassLastX = filteredX;
        lowPassLastY = filteredY;
        return new Translation2d(filteredX, filteredY);
    }

    /**
     * Update method that processes all filters and updates the filtered speed.
     * This should be called within the {@link edu.wpi.first.wpilibj2.command.Command#execute()} method of the command using this DriveInputProcessor.
     */
    public void update() {
        Translation2d driverInput = getDriverInputAsVelocity();
        
        Translation2d deadbanded = applyDeadband(driverInput);
        Translation2d powered = applyPowerCurve(deadbanded);
        Translation2d scaled = scaleToMaxVelocity(powered);
        Translation2d rateLimited = applyRateLimit(scaled);
        Translation2d completedFiltered = applyLowPassFilter(rateLimited);

        filteredSpeed = completedFiltered;
    }

    /**
     * Get the filtered speed.
     * @return A Translation2d representing the filtered speed
     */
    public Translation2d get() {
        return filteredSpeed;
    }
}
