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

    public static enum ShooterIOOutputMode {
        VELOCITY,
        STOP
    }

    public static class ShooterIOOutputs {
        public ShooterIOOutputMode mode = ShooterIOOutputMode.STOP;
        public AngularVelocity targetVelocity = RPM.zero();
        public int gainSlot = 0;
    }

    public default void applyOutputs(ShooterIOOutputs outputs) {}
}
