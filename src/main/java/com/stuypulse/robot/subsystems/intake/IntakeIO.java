package com.stuypulse.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.units.measure.*;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // Pivot
        public TalonFXInputs pivotMotorInputs = new TalonFXInputs();
        public boolean limitSwitchHit = false;
        public boolean pivotStalling = false;
        public boolean pivotPushingDown = false;

        // Roller
        public TalonFXInputs rollerMotorInputs = new TalonFXInputs();
        public boolean leftRollerStalling = false;
        public boolean rightRollerStalling = false;
    }

    public enum IntakeIOPivotOutputMode {
        STOP,
        POSITION,
        PUSHDOWN,
        HOMING,
    }

    public enum IntakeIORollerOutputMode {
        STOP,
        DUTY_CYCLE
    }

    public static class IntakeIOOutputs {
        public IntakeIOPivotOutputs pivot = new IntakeIOPivotOutputs();
        public IntakeIORollerOutputs roller = new IntakeIORollerOutputs();

        public static class IntakeIOPivotOutputs {
            @AutoLogOutput(key="Intake/Pivot/Output Mode")
            public IntakeIOPivotOutputMode outputMode = IntakeIOPivotOutputMode.STOP;

            @AutoLogOutput(key="Intake/Pivot/Position")
            public Angle position = Settings.Intake.Pivot.INITIAL_ANGLE;

            @AutoLogOutput(key="Intake/Pivot/Position Gains Slot")
            public int positionGainsSlot = 0;
            
            @AutoLogOutput(key="Intake/Pivot/Pushdown Current")
            public Current pushdown = Amps.of(0.0);

            @AutoLogOutput(key="Intake/Pivot/Homing Voltage")
            public Voltage homing = Volts.of(0.0);
            
            public Optional<Voltage> voltageOverride = Optional.empty();
        }

        public static class IntakeIORollerOutputs {
            @AutoLogOutput(key="Intake/Roller/Output Mode")
            public IntakeIORollerOutputMode outputMode = IntakeIORollerOutputMode.STOP;

            @AutoLogOutput(key="Intake/Roller/Target Duty Cycle")
            public double targetDutyCycle = 0.0;
        }
    }

    public default void updateInputs(IntakeIOInputs inputs) {};
    public default void applyOutputs(IntakeIOOutputs outputs) {};

    public default void seedPivotAngle(Angle angle) {};
}
