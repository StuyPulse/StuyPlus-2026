package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.*;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RPM.zero();
        public Voltage voltage = Volts.zero();
        public Current supplyCurrent = Amps.zero();
        public Current statorCurrent = Amps.zero();
        public boolean isStalling = false;
    }

    public default void updateInputs(HandoffIOInputs inputs) {};

    public static enum HandoffIOOutputMode {
        VOLTAGE,
        STOP
    }

    public static class HandoffIOOutputs {
        @AutoLogOutput(key = "Handoff/Mode")
        public HandoffIOOutputMode mode = HandoffIOOutputMode.STOP;

        @AutoLogOutput(key = "Handoff/Target Voltage")
        public Voltage voltage = Volts.zero();
    }

    public default void applyOutputs(HandoffIOOutputs outputs) {};
}
