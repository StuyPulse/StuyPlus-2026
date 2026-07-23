/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.simulation.TalonFXSimulation;

import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXSimulation extends TalonFX {
    private final SystemSim<?> simMotor;
    private final double gearRatio;

    public TalonFXSimulation(int port, double gearRatio, SystemSim<?> adapter) {
        super(port);
        this.gearRatio = gearRatio;
        this.simMotor = adapter;
    }

    public void configure(TalonFXConfig config) {
        config.configure(this);
    }

    public void refresh() {
        final TalonFXSimState simState = this.getSimState();

        this.simMotor.setInputVoltage(simState.getMotorVoltageMeasure());

        Angle rotorPosition = simMotor.getMechanismPositionRotations().times(this.gearRatio);
        AngularVelocity rotorVelocity = this.simMotor.getMechanismVelocity().times(this.gearRatio);

        simState.setRawRotorPosition(rotorPosition);
        simState.setRotorVelocity(rotorVelocity);
        simState.setSupplyVoltage(Volts.of(12.0));
    }
}
