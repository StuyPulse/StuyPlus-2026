package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public interface IntakeIO {
    public static class IntakeIOInputs {
        boolean isPivotStalling = false;
        boolean isLeftRollerStalling = false;
        boolean isRightRollerStalling = false;

        boolean isLimitSwitchHit = false;

        Angle relativePosition = Degrees.of(0.0);
        AngularVelocity rollerVelocity = RadiansPerSecond.of(0.0);

        boolean isApplyingPushdown = false;

        Angle pivotPosition = Degrees.of(0.0);
        Voltage pivotVoltage = Volts.of(0.0);
        AngularVelocity pivotVelocity = RadiansPerSecond.of(0.0);
    }
    void updateInputs(IntakeIOInputs inputs);

    void seedPivotAngle(Angle angle);

    void applyPushdown();
    void applyHoming();
    void applyPosition(Angle angle, int gainSlot);
    void applyRollerDutyCycle(double dutyCycle);
    
    void stopRollerMotors();
    void stopPivotMotor();
    default void stopAllMotors() {
        stopRollerMotors();
        stopPivotMotor();
    }

    default void logHardwareSignals() {};
    default void setVoltageOverride(Voltage voltage) {};
    default void updateSim() {};
}
