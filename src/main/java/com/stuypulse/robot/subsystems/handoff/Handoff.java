package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.Robot;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOInputs;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import dev.doglog.DogLog;

public class Handoff extends SubsystemBase {
    private static Handoff instance;

    private HandoffIO io;
    private HandoffIOInputs inputs;

    private HandoffState state;

    /** Enum representing the different possible states of the handoff. */
    public enum HandoffState {
        /** Handoff is stopped. */
        IDLE(Settings.Handoff.IDLE_VOLTAGE),
        /** The handoff runs forward. */
        FORWARD(Settings.Handoff.FORWARD_VOLTAGE),
        /** The handoff runs backward. */
        REVERSE(Settings.Handoff.REVERSE_VOLTAGE);

        /** The target voltage of the handoff motor. */
        private Voltage targetVoltage;

        /**
         * Constructs a HandoffState with the given target voltage.
         * @param targetVoltage the target voltage of the handoff motor in the corresponding state.
         */
        private HandoffState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        /**
         * Gets the target voltage of the handoff motor in the corresponding state.
         * @return the target voltage of the handoff motor
         */
        public Voltage getTargetVoltage() {
            return targetVoltage;
        }
    }

    static {
        instance = new Handoff();
    }

    public static Handoff getInstance() {
        return instance;
    }

    private Handoff() {
        setState(HandoffState.IDLE);

        this.inputs = new HandoffIOInputs();
        this.io = Robot.isReal()
            ? new HandoffIOImpl()
            : new HandoffIOSim();
    }

    public void setState(HandoffState state) {
        this.state = state;
    }

    public HandoffState getState() {
        return state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (!Settings.EnabledSubsystems.HANDOFF.get()) {
            io.stopMotors();
            return;
        }

        final Shooter shooter = Shooter.getInstance();
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        if (!(swerve.isAlignedToTarget(Field.getHubPose()))
                && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.IDLE);
        }
        // TODO: consider relaxing tolerances for ferrying
        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
                && shooter.getState() == ShooterState.FERRY) {
            setState(HandoffState.IDLE);
        }

        final Voltage targetVoltage = inputs.isStalling
            ? HandoffState.REVERSE.getTargetVoltage()
            : this.state.getTargetVoltage();

        io.setMotorVoltage(targetVoltage);

        
        DogLog.log("Handoff/Handoff Target Voltage", state.getTargetVoltage());
        DogLog.log("Handoff/Velocity", inputs.angularVelocity.in(RotationsPerSecond));
        DogLog.log("Handoff/Voltage", inputs.voltage.in(Volts));

        DogLog.log("Handoff/State", state.name());
        DogLog.forceNt.log("States/Handoff", state.name());
        DogLog.forceNt.log("Handoff/Stalling", inputs.isStalling);
        io.logHardwareSignals();
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }
}
