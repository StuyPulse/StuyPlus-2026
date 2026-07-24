/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandoffIOSim extends HandoffIOTalonFXBase {
    private static TalonFXSimulation getHandoffMotor(int id) {
        final DCMotorSim sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        Settings.Handoff.J_KG_METERS_SQUARED,
                        Settings.Handoff.GEAR_RATIO),
                DCMotor.getKrakenX60(1));
        final TalonFXSimulation motor = new TalonFXSimulation(id, sim);
        return motor;
    }

    private final TalonFXSimulation handoffMotor;

    public HandoffIOSim() {
        this(getHandoffMotor(Ports.Feeder.FEEDER_MOTOR));
    }

    private HandoffIOSim(TalonFXSimulation handoffMotor) {
        super(handoffMotor);
        this.handoffMotor = handoffMotor;
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        handoffMotor.update(Settings.DT);
        super.updateInputs(inputs);
    }
}
