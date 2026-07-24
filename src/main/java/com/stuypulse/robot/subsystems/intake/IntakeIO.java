package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // Pivot
        public Angle pivotPosition = Radians.zero();
        public AngularVelocity pivotVelocity = RPM.zero(); // for sysid
        public Voltage pivotVoltage = Volts.zero();
        public Current pivotSupplyCurrent = Amps.zero();
        public Current pivotStatorCurrent = Amps.zero();
        public boolean limitSwitchHit = false;
        public boolean pivotStalling = false;
        public boolean pivotPushingDown = false;

        // Roller
        public AngularVelocity rollerVelocity = RPM.zero();
        public Voltage rollerVoltage = Volts.zero();
        public Current rollerStatorCurrent = Amps.zero();
        public Current rollerSupplyCurrent = Amps.zero();
        public double rollerDutyCycle = 0.0;
        public boolean leftRollerStalling = false;
        public boolean rightRollerStalling = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void seedPivotAngle(Angle angle) {};

    public default void stopRollerMotors() {};
    public default void stopPivotMotor() {};
    public default void stopAllMotors() {
        stopRollerMotors();
        stopPivotMotor();
    };

    public default void setPivotPosition(Angle position) {};
    public default void setPivotPushdown(Current current) {};
    public default void setPivotHoming(Voltage voltage) {};
    public default void setVoltageOverride(Voltage voltage) {}

    public default void setRollerDutyCycle(double dutyCycle) {};
}
