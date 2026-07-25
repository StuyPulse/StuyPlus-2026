package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RPM.zero();
        public Voltage voltage = Volts.zero();
        public Current torqueCurrent = Amps.zero();
        public Current supplyCurrent = Amps.zero();
        public Current statorCurrent = Amps.zero();
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    public default void setGainsSlot(int slot) {};
    public default void setTargetVelocity(AngularVelocity velocity) {};
    public default void setTargetVoltage(Voltage voltage) {};

    public default void stopMotors() {};
}
