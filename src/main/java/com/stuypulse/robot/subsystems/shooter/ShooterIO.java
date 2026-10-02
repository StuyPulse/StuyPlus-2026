package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

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
        @AutoLogOutput(key = "Shooter/Mode")
        public ShooterIOOutputMode mode = ShooterIOOutputMode.STOP;

        @AutoLogOutput(key = "Shooter/Target Velocity")
        public AngularVelocity targetVelocity = RPM.zero();
        
        @AutoLogOutput(key = "Shooter/Gain Slot")
        public int gainSlot = 0;
    }

    public default void applyOutputs(ShooterIOOutputs outputs) {}
}
