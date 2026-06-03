package com.stuypulse.robot.util.swerveinput;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
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

    private LinearFilter lowPassFilter;

    private double intermediateProcessedAngularVelocity;

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

        this.lowPassFilter = LinearFilter.singlePoleIIR(this.rc, Settings.DT.in(Seconds));

        this.intermediateProcessedAngularVelocity = 0;
        this.filteredAngularVelocity = 0;
    }

    /**
     * Read the raw joystick axis value for the right X axis and store it in {@link #intermediateProcessedAngularVelocity}
     * @return This instance of the class
     */
    private DriveTurnInputProcessor getRightX() {
        this.intermediateProcessedAngularVelocity = controller.getRightX();
        return this;
    }

    /**
     * Applies a deadband to the current {@link #intermediateProcessedAngularVelocity} value.
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyDeadband() {
        this.intermediateProcessedAngularVelocity = MathUtil.applyDeadband(this.intermediateProcessedAngularVelocity, deadband);
        return this;
    }

    /**
     * Apply a power curve to the current {@link #intermediateProcessedAngularVelocity} value.
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyPowerCurve() {
        this.intermediateProcessedAngularVelocity = Math.pow(Math.abs(this.intermediateProcessedAngularVelocity), power) * Math.signum(this.intermediateProcessedAngularVelocity);
        return this;
    }

    /**
     * Scale the {@link #intermediateProcessedAngularVelocity} value to the robot's maximum velocity.
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyScalingToMaxAngularVelocity() {
        this.intermediateProcessedAngularVelocity = this.intermediateProcessedAngularVelocity * maxAngularVelocity;
        return this;
    }

    /**
     * Applies a low pass filter to {@link #intermediateProcessedAngularVelocity} using
     * a {@link edu.wpi.first.math.filter.LinearFilter}.
     * This smooths out the input and reduces noise.
     * 
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyLowPassFilter() {
        this.intermediateProcessedAngularVelocity = lowPassFilter.calculate(this.intermediateProcessedAngularVelocity);
        return this;
    }

    /**
     * Update method that processes all filters and updates the filtered angular velocity.
     * This should be called within the {@link edu.wpi.first.wpilibj2.command.Command#execute()} method of the command using this DriveTurnInputProcessor.
     */
    public void update() {
        getRightX()
            .applyDeadband()
            .applyPowerCurve()
            .applyScalingToMaxAngularVelocity()
            .applyLowPassFilter();

        this.filteredAngularVelocity = this.intermediateProcessedAngularVelocity;
    }

    /**
     * Get the processed angular velocity
     * @return The filtered angular velocity
     */
    public double get() {
        return filteredAngularVelocity;
    }
}
