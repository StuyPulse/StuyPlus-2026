/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim extends FeederIOBase {
    private static final SystemSim<DCMotorSim> sim = SystemSim.of(new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                FeederConstants.FeederSettings.J.in(KilogramSquareMeters),
                FeederConstants.FeederSettings.GEAR_RATIO),
            DCMotor.getKrakenX60(2)));
    private static TalonFXSimulation getFeederMotor(int id) {
        final TalonFXSimulation motor = new TalonFXSimulation(id, FeederConstants.FeederSettings.GEAR_RATIO, sim);
        return motor;
    }

    private final TalonFXSimulation feederMotor;

    public FeederIOSim() {
        this(getFeederMotor(FeederConstants.FeederDeviceIds.FEEDER_MOTOR));
    }

    private FeederIOSim(TalonFXSimulation feederMotor) {
        super(feederMotor);
        this.feederMotor = feederMotor;
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        sim.update(Settings.DT);
        feederMotor.refresh();

        super.updateInputs(inputs);
        RobotVisualizer.getInstance().updateFeeder(inputs.feederMotorInputs.velocity);
    }
}