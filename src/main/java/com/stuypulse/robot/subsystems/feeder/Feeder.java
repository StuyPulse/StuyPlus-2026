/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.robot.util.simulation.RobotVisualizer;

public class Feeder extends SubsystemBase {
    private static final Feeder instance;

    static {
        instance = Robot.isReal() ? new Feeder(new FeederIOTalonFX()) : new Feeder(new FeederIOSim());
    }

    public static Feeder getInstance() {
        return instance;
    }

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs;
    private FeederState state;

    private Feeder(FeederIO io) {
        this.io = io;
        this.inputs = new FeederIOInputsAutoLogged();
        this.state = FeederState.IDLE;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public FeederState getState() {
        return state;
    }

    public enum FeederState {
        IDLE(Volts.of(0.0)),
        FORWARD(Settings.Feeder.FORWARD_VOLTAGE),
        REVERSE(Settings.Feeder.REVERSE_VOLTAGE);

        private final Voltage targetVoltage;

        private FeederState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        public Voltage getTargetVoltage() {
            return this.targetVoltage;
        }
    }

    private Command setStateCommand(FeederState state) {
        return Commands.runOnce(() -> this.setState(state));
    }

    public Command setIdle() {
        return setStateCommand(FeederState.IDLE);
    }

    public Command setForward() {
        return setStateCommand(FeederState.FORWARD);
    }

    public Command setReverse() {
        return setStateCommand(FeederState.REVERSE);
    }

    @Override
    public void periodic() {
        final FeederState currentState = this.getState();
        // Stop shooting if not aligned
        // final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        // final Shooter shooter = Shooter.getInstance();
        // if (!(swerve.isAlignedToTarget(Field.getHubPose()))
        //         && shooter.getState() == ShooterState.SHOOT) {
        //     setState(FeederState.IDLE);
        // }
        // if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
        //         && shooter.getState() == ShooterState.FERRY) {
        //     setState(FeederState.IDLE);
        // }

        if (Settings.EnabledSubsystems.FEEDER.get()) {
            io.setTargetVoltage(currentState.getTargetVoltage());
        } else {
            io.stopMotors();
        }
        io.updateInputs(inputs);
        RobotVisualizer.getInstance().updateFeeder(inputs.velocity);

        // Logging
        DogLog.log("Feeder/State", currentState.name());
        DogLog.forceNt.log("States/Feeder", currentState.name());
    }
}
