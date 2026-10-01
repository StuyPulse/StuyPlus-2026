/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOOutputMode;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOOutputs;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.simulation.RobotVisualizer;

public class Feeder extends FullSubsystem {
    private static final Feeder instance;

    static {
        switch (Settings.CURRENT_MODE) {
            case REAL -> instance = new Feeder(new FeederIOTalonFX());

            case SIM -> instance = new Feeder(new FeederIOSim());

            default -> instance = new Feeder(new FeederIO() {});
        }
    }

    public static Feeder getInstance() {
        return instance;
    }

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs;
    private final FeederIOOutputs outputs;

    @AutoLogOutput(key = "States/Feeder")
    private FeederState state;

    private Feeder(FeederIO io) {
        this.io = io;
        this.inputs = new FeederIOInputsAutoLogged();
        this.outputs = new FeederIOOutputs();
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
        FORWARD(FeederConstants.FeederSettings.FORWARD_VOLTAGE),
        REVERSE(FeederConstants.FeederSettings.REVERSE_VOLTAGE);

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
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
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
            runVoltage(getState().getTargetVoltage());
        } else {
            outputs.mode = FeederIOOutputMode.STOP;
        }
        
        if (!Robot.isReal()) {
            RobotVisualizer.getInstance().updateFeeder(inputs.velocity);
        }
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private void runVoltage(Voltage voltage) {
        outputs.mode = FeederIOOutputMode.VOLTAGE;
        outputs.voltage = voltage;
    }
}
