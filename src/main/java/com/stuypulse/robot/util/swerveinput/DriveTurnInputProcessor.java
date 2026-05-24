package com.stuypulse.robot.util.swerveinput;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

/**
 * <h2>DriveTurnInputProcessor</h2>
 * <p>Class for processing driver turn input for the drivetrain. It's intended to replace StuyLib's I3Stream 
 * and Filters and use pure WPILIB and Java.</p>
 * <p>This will be used to be as close as the original StuyLib implementation as possible</p>
 * 
 * It will be used for
 * <ul>
 *   <li>Applying deadbands</li>
 *   <li>Applying power curves</li>
 *   <li>Applying max angular velocity</li>
 *   <li>Applying a low pass filter</li>
 * </ul>
 * 
 * <p>To use within a command, create an instance of this class and call the {@link #update()} method in the {@link edu.wpi.first.wpilibj2.command.Command#execute()} method of the command.</p>
 * <p>To get the processed speed, call the {@link #get()} method.</p>
 */
public class DriveTurnInputProcessor {
    private CommandXboxController controller;
    private double deadband;
    private double power;
    private double maxAngularVelocity; // radians/s
    private double rc; // low pass filter time constant

    private double lowPassLastAngularVelocity;

    private double filteredAngularVelocity;

    /**
     * <h4>Constructor for the DriveInputProcessor</h4>
     * <p>Creates a new DriveInputProcessor with the specified parameters and creates a notifier to run periodically (20ms) and process the input.</p>
     * @param controller The CommandXboxController to get driver input from
     * @param deadband The deadband to apply to the input (0-1)
     * @param power The power to apply to the input
     * @param maxAngularVelocity The maximum angular velocity to scale the input to (radians/s)
     * @param rc The time constant for the low pass filter (seconds)
     */
    public DriveTurnInputProcessor(CommandXboxController controller, double deadband, double power, double maxAngularVelocity, double rc) {
        this.controller = controller;
        this.deadband = deadband;
        this.power = power;
        this.maxAngularVelocity = maxAngularVelocity;
        this.rc = rc;

        this.lowPassLastAngularVelocity = 0;
    }

    /**
     * Get the right X input from the controller.
     * @return The right X input value
     */
    private double getRightX() {
        return controller.getRightX();
    }

    /**
     * Applies a deadband to the given input value.
     * @param input The input value to apply the deadband to
     * @return The input value with the deadband applied
     */
    private double applyDeadband(double input) {
        return MathUtil.applyDeadband(input, deadband);
    }

    /**
     * Applies a power curve to the given input value.
     * @param input The input value to apply the power curve to
     * @return The input value with the power curve applied
     */
    private double applyPowerCurve(double input) {
        return Math.pow(Math.abs(input), power) * Math.signum(input);
    }

    /**
     * Scales the given input value to the maximum angular velocity.
     * @param input The input value to scale to the maximum angular velocity
     * @return The input value scaled to the maximum angular velocity
     */
    private double scaleToMaxAngularVelocity(double input) {
        return input * maxAngularVelocity;
    }

    /**
     * Applies a low pass filter to the given input value.
     * @param input The input value to apply the low pass filter to
     * @return The input value with the low pass filter applied
     */
    private double applyLowPassFilter(double input) {
        if (rc <= 0) {
            lowPassLastAngularVelocity = input;
            return input;
        }

        double alpha = 1.0 - Math.exp(-Settings.DT.in(Seconds) / rc);
        double filtered = lowPassLastAngularVelocity + alpha * (input - lowPassLastAngularVelocity);
        lowPassLastAngularVelocity = filtered;
        return filtered;
    }

    /**
     * Update method that processes all filters and updates the filtered angular velocity.
     * This should be called within the {@link edu.wpi.first.wpilibj2.command.Command#execute()} method of the command using this DriveTurnInputProcessor.
     */
    public void update() {
        double rightX = getRightX();
        double deadbanded = applyDeadband(rightX);
        double powered = applyPowerCurve(deadbanded);
        double scaled = scaleToMaxAngularVelocity(powered);
        double completedFiltered = applyLowPassFilter(scaled);

        filteredAngularVelocity = completedFiltered;
    }

    /**
     * Get the processed angular velocity
     * @return The filtered angular velocity
     */
    public double get() {
        return filteredAngularVelocity;
    }
}
