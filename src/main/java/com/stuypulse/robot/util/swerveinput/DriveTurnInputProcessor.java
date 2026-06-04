package com.stuypulse.robot.util.swerveinput;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

/**
 * <h2>DriveTurnInputProcessor</h2>
 * <p>Class for processing driver turn input for the drivetrain. It's intended to replace StuyLib's IStream 
 * and Filters and use pure WPILib and Java.</p>
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
    private final CommandXboxController controller;
    private final double deadband;
    private final double power;
    /** The max angular velocity in radians per second */
    private final double maxAngularVelocity;
    /** The time constant for the low pass filter (seconds) */
    private final double rc;

    private final LinearFilter lowPassFilter;

    /** The processed angular velocity after the full processing and filtering. */
    private double processedAngularVelocity;

    /**
     * <h4>Constructor for the DriveInputProcessor</h4>
     * <p>Creates a new DriveInputProcessor with the specified parameters processes the input periodically.</p>
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

        this.processedAngularVelocity = 0;
    }

    /**
     * Read the raw joystick axis value for the right X axis and store it in {@link #processedAngularVelocity}
     * @return This instance of the class
     */
    private DriveTurnInputProcessor getRightX() {
        this.processedAngularVelocity = controller.getRightX();
        return this;
    }

    /**
     * Applies a deadband to the current {@link #processedAngularVelocity} value.
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyDeadband() {
        this.processedAngularVelocity = MathUtil.applyDeadband(this.processedAngularVelocity, deadband);
        return this;
    }

    /**
     * Apply a power curve to the current {@link #processedAngularVelocity} value.
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyPowerCurve() {
        this.processedAngularVelocity = Math.pow(Math.abs(this.processedAngularVelocity), power) * Math.signum(this.processedAngularVelocity);
        return this;
    }

    /**
     * Scale the {@link #processedAngularVelocity} value to the robot's maximum velocity.
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyScalingToMaxAngularVelocity() {
        this.processedAngularVelocity = this.processedAngularVelocity * maxAngularVelocity;
        return this;
    }

    /**
     * Applies a low pass filter to {@link #processedAngularVelocity} using
     * a {@link edu.wpi.first.math.filter.LinearFilter}.
     * This smooths out the input and reduces noise.
     * 
     * @return This instance of the class
     */
    private DriveTurnInputProcessor applyLowPassFilter() {
        this.processedAngularVelocity = lowPassFilter.calculate(this.processedAngularVelocity);
        return this;
    }

    /**
     * Update method that processes all filters and updates the filtered angular velocity.
     * This should be called within the {@link edu.wpi.first.wpilibj2.command.Command#execute()} 
     * method of the command using this DriveTurnInputProcessor.
     */
    public void update() {
        getRightX()
            .applyDeadband()
            .applyPowerCurve()
            .applyScalingToMaxAngularVelocity()
            .applyLowPassFilter();
    }

    /**
     * Get the processed angular velocity
     * @return The processed angular velocity
     */
    public double get() {
        return processedAngularVelocity;
    }
}
