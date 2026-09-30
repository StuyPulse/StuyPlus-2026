/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.feeder.FeederConstants;
import com.stuypulse.robot.util.simulation.TalonSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandoffIOSim extends HandoffIOTalonFXBase {
    private static final SystemSim<DCMotorSim> sim = SystemSim.of(new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        HandoffConstants.HandoffSettings.J_KG_METERS_SQUARED,
                        HandoffConstants.HandoffSettings.GEAR_RATIO),
                DCMotor.getKrakenX60(1)));
    private static TalonFXSimulation getHandoffMotor(int id) {
        final TalonFXSimulation motor = new TalonFXSimulation(id, HandoffConstants.HandoffSettings.GEAR_RATIO, sim);
        return motor;
    }

    private final TalonFXSimulation handoffMotor;

    public HandoffIOSim() {
        this(getHandoffMotor(FeederConstants.FeederDeviceIds.FEEDER_MOTOR));
    }

    private HandoffIOSim(TalonFXSimulation handoffMotor) {
        super(handoffMotor);
        this.handoffMotor = handoffMotor;
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        sim.update(Settings.DT);
        handoffMotor.refresh();
        super.updateInputs(inputs);
    }
}
