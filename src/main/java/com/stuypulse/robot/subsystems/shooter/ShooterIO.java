package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;

import edu.wpi.first.units.measure.*;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public TalonFXInputs shooterMotorLeftInputs = new TalonFXInputs();
        public TalonFXInputs shooterMotorRightInputs = new TalonFXInputs();
        public TalonFXInputs shooterMotorCenterInputs = new TalonFXInputs();
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
