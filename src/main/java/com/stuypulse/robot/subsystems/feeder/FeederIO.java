package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RPM.zero();
        public Voltage voltage = Volts.zero();
        public Current current = Amps.zero();
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(FeederIOInputs inputs) {}

    public default void setTargetVoltage(Voltage voltage) {}

    public default void stopMotors() {}
}
