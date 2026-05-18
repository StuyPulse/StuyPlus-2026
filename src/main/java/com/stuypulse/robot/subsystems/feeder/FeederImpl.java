/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.LoggedSignals;
import edu.wpi.first.units.measure.*;

public class FeederImpl extends Feeder {

    private final TalonFX feederMotor;

    private final DutyCycleOut controller;

    private final LoggedSignals signals;

    public FeederImpl() {
        feederMotor = new TalonFX(Ports.Feeder.FEEDER_MOTOR, Settings.CANBUS);
        Motors.Feeder.LEADER_CONFIG.configure(feederMotor);
        controller = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);
        this.signals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Feeder/",
                feederMotor.getSupplyCurrent(),
                feederMotor.getStatorCurrent(),
                feederMotor.getVelocity());
    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return feederMotor.getVelocity().getValue();
    }

    @Override
    protected void stopMotors() {
        feederMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (!EnabledSubsystems.FEEDER.get()) {
            stopMotors();
            return;
        }
        // Stop shooting if not aligned
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
        // Apply
        feederMotor.setControl(controller.withOutput(getState().getTargetDutyCycle()));
        // Logging
        this.signals.logAll();
        super.periodic();
    }
}
