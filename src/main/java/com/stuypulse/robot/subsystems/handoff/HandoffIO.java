package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RPM.zero();
        public Voltage voltage = Volts.zero();
        public Current supplyCurrent = Amps.zero();
        public boolean isStalling = false;
    }

    public default void updateInputs(HandoffIOInputs inputs) {}

    public default void setTargetVoltage(Voltage voltage) {}

    public default void stopMotors() {}
}
