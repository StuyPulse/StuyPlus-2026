/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandoffSim extends Handoff {

    // private final TalonFXSimulation handoffMotor;

    // private final DCMotorSim handoffSim;

    // private final VoltageOut handoffMotorController;

    public HandoffSim() {
        // handoffSim = new DCMotorSim(
        //         LinearSystemId.createDCMotorSystem(
        //                 DCMotor.getKrakenX60(1),
        //                 Settings.Handoff.J_KG_METERS_SQUARED,
        //                 Settings.Handoff.GEAR_RATIO),
        //         DCMotor.getKrakenX60(1));
        // handoffMotor = new TalonFXSimulation(Ports.Handoff.HANDOFF_MOTOR, handoffSim);
        // handoffMotor.configure(Motors.Handoff.HANDOFF_MOTOR_CONFIG);
        // handoffMotorController = new VoltageOut(getState().getTargetVoltage()).withEnableFOC(true);
    }

    @Override
    protected void stopMotors() {
        // handoffMotor.stopMotor();
    }

    @Override
    protected boolean handoffStalling() {
        return false; // sim doesn't stall
    };

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.HANDOFF.get()) {
            Shooter shooter = Shooter.getInstance();
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
            if (!(swerve.isAlignedToTarget(Field.getHubPose()))
                    && shooter.getState() == ShooterState.SHOOT) {
                setState(HandoffState.IDLE);
            }
            if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
                    && shooter.getState() == ShooterState.FERRY) {
                setState(HandoffState.IDLE);
            }
            
            // handoffMotor.setControl(handoffMotorController.withOutput(getState().getTargetVoltage()));
        } else {
            stopMotors();
        }
        // handoffMotor.update(Settings.DT);
        super.periodic();
    }
}
