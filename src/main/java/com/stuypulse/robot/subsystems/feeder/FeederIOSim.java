/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim extends FeederIOTalonFXBase {
    private final TalonFXSimulation feederMotor;

    private static TalonFXSimulation getFeederMotor() {
        final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                Settings.Feeder.J.in(KilogramSquareMeters),
                Settings.Feeder.GEAR_RATIO),
            DCMotor.getKrakenX60(2));
        final TalonFXSimulation motor = new TalonFXSimulation(Ports.Feeder.FEEDER_MOTOR, sim);
        Motors.Feeder.LEADER_CONFIG.configure(motor);
        return motor;
    }

    public FeederIOSim() {
        this(getFeederMotor());
    }

    private FeederIOSim(TalonFXSimulation feederMotor) {
        super(feederMotor);
        this.feederMotor = feederMotor;
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederMotor.update(Settings.DT);
        super.updateInputs(inputs);
    }
}