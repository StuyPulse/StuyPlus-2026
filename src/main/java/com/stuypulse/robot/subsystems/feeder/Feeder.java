package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.Robot;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOInputs;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.simulation.TalonFXSimIds;

import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private static Feeder instance;

    private FeederIOInputs inputs;
    private FeederIO io;

    private FeederState state;

    /** Enum representing the different possible states of the feeder. */
    public enum FeederState {
        /** Feeder is stopped. */
        IDLE(Volts.of(0.0)),
        /** Motors run forward to feed to the shooter. */
        FORWARD(Settings.Feeder.FORWARD_VOLTAGE),
        /** Motors run backward to work with the intake to outtake fuel from the robot. */
        REVERSE(Settings.Feeder.REVERSE_VOLTAGE);

        /** The target voltage of the feeder motors. */
        private Voltage targetVoltage;

        /**
         * Constructs a FeederState with the given target voltage.
         * @param targetVoltage the target voltage of the feeder motors in the corresponding state.
         */
        private FeederState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        /** 
         * Gets the target voltage of the feeder motors in the corresponding state.
         * @return the target voltage of the feeder motors
         */
        public Voltage getTargetVoltage() {
            return this.targetVoltage;
        }
    }

    static {
        instance = new Feeder();
    }

    public static Feeder getInstance() {
        return instance;
    }

    private Feeder() {
        setState(FeederState.IDLE);

        this.inputs = new FeederIOInputs();
        this.io = Robot.isReal() 
            ? new FeederIOImpl(Ports.Feeder.FEEDER_MOTOR, Settings.CANBUS)
            : new FeederIOSim(TalonFXSimIds.get(), Settings.Feeder.GEAR_RATIO);
    }

    public AngularVelocity getCurrentAngularVelocity() {
        return inputs.angularVelocity;
    }

    public FeederState getState() {
        return state;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (!Settings.EnabledSubsystems.FEEDER.get()) {
            io.stopMotors();
            return;
        }

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        Shooter shooter = Shooter.getInstance();
        if (!(swerve.isAlignedToTarget(Field.getHubPose()))
                && shooter.getState() == ShooterState.SHOOT) {
            setState(FeederState.IDLE);
        }
        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
                && shooter.getState() == ShooterState.FERRY) {
            setState(FeederState.IDLE);
        }

        io.setMotorVoltage(getState().getTargetVoltage());
        
        DogLog.log("Feeder/Feeder Target Voltage", state.getTargetVoltage());
        DogLog.log("Feeder/Velocity", inputs.angularVelocity.in(RotationsPerSecond));
        DogLog.log("Feeder/Voltage", inputs.voltage.in(Volts));
        
        DogLog.log("Feeder/State", state.name());
        DogLog.forceNt.log("States/Feeder", state.name());
        io.logHardwareSignals();
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }
}
