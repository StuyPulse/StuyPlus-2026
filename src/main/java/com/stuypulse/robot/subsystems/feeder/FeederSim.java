/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederSim extends Feeder {

    private final TalonFXSimulation feederMotor;

    private final DCMotorSim feederSim;

    private final VoltageOut feederController;

    public FeederSim() {
        feederSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(2),
                        Settings.Feeder.J.in(KilogramSquareMeters),
                        Settings.Feeder.GEAR_RATIO),
                DCMotor.getKrakenX60(2));
        feederMotor = new TalonFXSimulation(Ports.Feeder.FEEDER_MOTOR, feederSim);
        feederMotor.configure(Motors.Feeder.LEADER_CONFIG);
        feederController = new VoltageOut(0).withEnableFOC(true);
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
        if (!Settings.EnabledSubsystems.FEEDER.get()) {
            stopMotors();
            return;
        }
        // apply control to leader before grabbing state to update other motors
        feederMotor.setControl(feederController.withOutput(getState().getTargetVoltage()));
        feederMotor.update(Settings.DT);
        RobotVisualizer.getInstance().updateFeeder(getCurrentAngularVelocity());
        // Logging
        super.periodic();
    }
}
